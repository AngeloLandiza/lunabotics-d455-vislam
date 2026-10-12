#!/usr/bin/env python3
"""Minimal local server for NVIDIA Cosmos 3 Edge (reasoner: image + text -> text).

Speaks the subset of the OpenAI chat-completions API that sim/cosmos_agent.py uses, the
same API as the model card's vLLM server, so either can sit behind the agent:

    POST /v1/chat/completions  {"messages": [...], "max_tokens": N, "temperature": T,
                                "chat_template_kwargs": {"enable_thinking": bool}}
    -> {"choices": [{"message": {"role": "assistant", "content": answer,
                                 "reasoning_content": thinking}}], "usage": {...}}

Images come as {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}}.
Extension: "thinking_budget": N caps the reasoning; at N tokens the reply is forced to
"</think>" + "answer_prefix" (e.g. "ACTION:") and gets "answer_tokens" more to finish.

Measured on an 8 GB RTX 4060 laptop (sim/cosmos_bench.py, ~1100-token prompt; the agent answers
directly on routine steps, so prefill matters as much as decode speed):
    --quant int8  torchao int8 weight-only + torch.compile   prefill ~0.2 s, ~95 tok/s, 3.2 GB  (default)
    --quant int4  torchao int4 (tinygemm) + torch.compile    prefill ~0.55 s, ~110 tok/s, 2.4 GB
    --quant bf16  official weights, the quality reference    prefill ~0.18 s, ~57 tok/s, 4.9 GB
    --quant nf4   bitsandbytes 4-bit community build          prefill ~0.19 s, ~71 tok/s, 2.4 GB
int8 / int4 are quantized from nvidia/Cosmos3-Edge at load time and compile the decoder on start
(about a minute the first time; torch caches it in /tmp/torchinductor_$USER).

While generating it writes the request and the streamed reply to --live-dir (image.jpg,
prompt.txt, output.txt, state.json) for sim/cosmos_viewer.py.
Runs in ~/cosmos-env (sim/setup_cosmos.sh), not in the ROS Python:
    ~/cosmos-env/bin/python sim/cosmos_server.py [--quant int8] [--port 8000]
"""
import argparse
import base64
import io
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import torch
from PIL import Image
from transformers import (AutoModelForImageTextToText, AutoProcessor, LogitsProcessor, LogitsProcessorList,
                          StaticCache, StoppingCriteria, StoppingCriteriaList, TextIteratorStreamer)

parser = argparse.ArgumentParser()
parser.add_argument('--quant', default='int8', choices=['int8', 'int4', 'bf16', 'nf4'])
parser.add_argument('--port', type=int, default=8000)
parser.add_argument('--cache-sizes', default='1792,2560',
                    help='static KV cache lengths (prompt + reply); each request uses the smallest that fits')
parser.add_argument('--live-dir', default='~/.ros/cosmos_runs/live', help='live output for sim/cosmos_viewer.py')
args = parser.parse_args()
LIVE = os.path.expanduser(args.live_dir)
os.makedirs(LIVE, exist_ok=True)

t0 = time.time()
if args.quant == 'nf4':
    MODEL_ID, kw = 'ubr-physical-ai/Cosmos3-Edge-NF4-bnb', {}
else:
    MODEL_ID, kw = 'nvidia/Cosmos3-Edge', {}
    if args.quant in ('int4', 'int8'):
        from torchao.quantization import Int4WeightOnlyConfig, Int8WeightOnlyConfig
        from transformers import TorchAoConfig
        cfg = (Int4WeightOnlyConfig(group_size=128, int4_packing_format='tile_packed_to_4d')
               if args.quant == 'int4' else Int8WeightOnlyConfig())
        # The vision encoder runs once per request (prefill): keep it in bf16.
        kw['quantization_config'] = TorchAoConfig(quant_type=cfg, modules_to_not_convert=['model.visual',
                                                                                         'model.projector'])
model = AutoModelForImageTextToText.from_pretrained(MODEL_ID, device_map='cuda', dtype=torch.bfloat16, **kw).eval()
processor = AutoProcessor.from_pretrained(MODEL_ID)
tok = processor.tokenizer
# Compiled decoding needs a static KV cache of a fixed size, and every decode step attends over
# all of it: 3072 slots ran int4 at 78 tok/s, 1536 at 112. So keep a few sizes (each compiles
# once) and use the smallest that fits the request; prompts are ~1100-1450 tokens (+128 reasoning).
COMPILE = args.quant in ('int4', 'int8')
caches = ([StaticCache(config=model.config, max_cache_len=int(n)) for n in sorted(map(int, args.cache_sizes.split(',')))]
          if COMPILE else [])
lock = threading.Lock()   # one generation at a time on the GPU
print(f'[cosmos_server] {MODEL_ID} ({args.quant}) loaded in {time.time() - t0:.0f} s, '
      f'{torch.cuda.memory_allocated() / 1e9:.1f} GB on the GPU', flush=True)


def to_hf(messages):
    """OpenAI-style messages -> transformers chat messages (data-URI images -> PIL)."""
    out = []
    for m in messages:
        content = m['content']
        if isinstance(content, str):
            content = [{'type': 'text', 'text': content}]
        parts = []
        for c in content:
            if c['type'] == 'image_url':
                data = c['image_url']['url'].split(',', 1)[1]
                parts.append({'type': 'image', 'image': Image.open(io.BytesIO(base64.b64decode(data))).convert('RGB')})
            else:
                parts.append(c)
        out.append({'role': m['role'], 'content': parts})
    return out


class ThinkingBudget(LogitsProcessor):
    """Budget forcing in one generate() pass: after `budget` reasoning tokens, emit the forced
    tokens ("\\n</think>\\n\\n" + answer prefix) one by one, then allow `answer_tokens` more."""

    def __init__(self, n_prompt, budget, forced, answer_tokens):
        self.n_prompt, self.budget, self.answer_tokens = n_prompt, budget, answer_tokens
        self.forced = tok.encode(forced, add_special_tokens=False)
        self.queue, self.answer_start = None, None

    def __call__(self, input_ids, scores):
        n = input_ids.shape[1] - self.n_prompt
        if self.answer_start is None and self.queue is None:
            if '</think>' in tok.decode(input_ids[0, -8:]):   # the model closed its reasoning itself
                self.answer_start = n
            elif n >= self.budget:
                self.queue = list(self.forced)
        if self.queue:
            forced = self.queue.pop(0)
            scores = torch.full_like(scores, -float('inf'))
            scores[:, forced] = 0
            if not self.queue:
                self.answer_start = n + 1
        return scores


class AnswerDone(StoppingCriteria):
    def __init__(self, budget):
        self.b = budget

    def __call__(self, input_ids, scores, **kw):
        n = input_ids.shape[1] - self.b.n_prompt
        done = self.b.answer_start is not None and n - self.b.answer_start >= self.b.answer_tokens
        return torch.full((input_ids.shape[0],), done, dtype=torch.bool, device=input_ids.device)


def live(name, data):
    """Atomic write into the live dir, so the viewer never reads half a file."""
    path = os.path.join(LIVE, name)
    with open(path + '.tmp', 'wb' if isinstance(data, bytes) else 'w') as f:
        f.write(data)
    os.replace(path + '.tmp', path)


def show_request(req, n):
    user = [m for m in req['messages'] if m['role'] == 'user'][-1]['content']
    for c in user if isinstance(user, list) else [{'type': 'text', 'text': user}]:
        if c['type'] == 'image_url':
            live('image.jpg', base64.b64decode(c['image_url']['url'].split(',', 1)[1]))
        elif c['type'] == 'text':
            live('prompt.txt', c['text'])
    live('output.txt', '')
    live('state.json', json.dumps({'request': n, 'state': 'thinking', 'started': time.time()}))


def generate(stream, **kw):
    """model.generate, with the reply streamed into output.txt for the viewer."""
    if not stream:
        return model.generate(**kw)
    streamer = TextIteratorStreamer(tok, skip_prompt=True, skip_special_tokens=True)
    out = {}

    def run():
        try:
            with torch.inference_mode():   # thread-local: the caller's inference mode doesn't reach here
                out['ids'] = model.generate(**kw, streamer=streamer)
        except Exception as e:
            out['error'] = e
            streamer.end()

    worker = threading.Thread(target=run)
    worker.start()
    text, last = '', 0.0
    for piece in streamer:
        text += piece
        if time.time() - last > 0.15:
            live('output.txt', text)
            last = time.time()
    worker.join()
    live('output.txt', text)
    if 'error' in out:
        raise out['error']
    return out['ids']


def complete(req, stream=True, use_cache=None):
    think = req.get('chat_template_kwargs', {}).get('enable_thinking', True)
    inputs = processor.apply_chat_template(to_hf(req['messages']), add_generation_prompt=True, tokenize=True,
                                           return_dict=True, return_tensors='pt',
                                           enable_thinking=think).to(model.device)
    n_prompt = inputs['input_ids'].shape[1]
    temperature = float(req.get('temperature', 0.6))
    gen = dict(do_sample=temperature > 0, max_new_tokens=int(req.get('max_tokens', 1024)))
    if temperature > 0:
        gen.update(temperature=temperature, top_p=float(req.get('top_p', 0.95)))
    budget = req.get('thinking_budget')
    if think and budget:
        forcer = ThinkingBudget(n_prompt, int(budget), '\n</think>\n\n' + req.get('answer_prefix', ''),
                                int(req.get('answer_tokens', 48)))
        gen.update(logits_processor=LogitsProcessorList([forcer]),
                   stopping_criteria=StoppingCriteriaList([AnswerDone(forcer)]),
                   max_new_tokens=int(budget) + len(forcer.forced) + forcer.answer_tokens + 8)
    cache = None
    if COMPILE:
        need = n_prompt + gen['max_new_tokens'] + 1
        cache = use_cache or next((c for c in caches if c.max_cache_len >= need), caches[-1])
        gen['max_new_tokens'] = min(gen['max_new_tokens'], cache.max_cache_len - n_prompt - 1)
        gen['past_key_values'] = cache
    with lock, torch.inference_mode():
        if cache is not None:
            cache.reset()   # inside inference mode: the cache tensors were created in it
        ids = generate(stream, **inputs, **gen)
    new = ids[0, n_prompt:]
    text = processor.decode(new, skip_special_tokens=True)
    # Thinking on: the prompt already opened <think>, so the reply is "reasoning</think>answer".
    reasoning, answer = (text.split('</think>', 1) if '</think>' in text else ('', text)) if think else ('', text)
    return {'choices': [{'message': {'role': 'assistant', 'content': answer.strip(),
                                     'reasoning_content': reasoning.replace('<think>', '').strip()},
                         'finish_reason': 'stop'}],
            'usage': {'prompt_tokens': int(n_prompt), 'completion_tokens': int(len(new))}}


def warm_up():
    """Compile the decoder before the first real request (int4/int8: ~1 min the first time)."""
    t = time.time()
    buf = io.BytesIO()
    Image.new('RGB', (512, 320), (90, 90, 90)).save(buf, format='PNG')
    img = {'type': 'image_url', 'image_url': {'url': 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()}}
    for c in caches or [None]:   # compile the decoder for each cache size
        for _ in range(2):
            complete({'messages': [{'role': 'user', 'content': [img, {'type': 'text', 'text': 'Describe it briefly.'}]}],
                      'max_tokens': 24, 'temperature': 0, 'chat_template_kwargs': {'enable_thinking': False}},
                     stream=False, use_cache=c)
    print(f'[cosmos_server] warm-up done in {time.time() - t:.0f} s', flush=True)


class Handler(BaseHTTPRequestHandler):
    count = 0

    def do_GET(self):   # health check / model list
        self.reply(200, {'data': [{'id': MODEL_ID, 'quant': args.quant}]})

    def do_POST(self):
        try:
            req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            t = time.time()
            Handler.count += 1
            show_request(req, Handler.count)
            res = complete(req)
            u = res['usage']
            u['seconds'] = time.time() - t
            live('state.json', json.dumps({'request': Handler.count, 'state': 'done', 'seconds': u['seconds'],
                                           'tokens': u['completion_tokens'],
                                           'answer': res['choices'][0]['message']['content']}))
            print(f'[cosmos_server] {u["prompt_tokens"]} -> {u["completion_tokens"]} tokens '
                  f'in {u["seconds"]:.1f} s', flush=True)
            self.reply(200, res)
        except Exception as e:   # report to the client instead of dropping the connection
            live('state.json', json.dumps({'request': Handler.count, 'state': 'error', 'answer': repr(e)}))
            self.reply(500, {'error': repr(e)})

    def reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


warm_up()
print(f'[cosmos_server] listening on http://127.0.0.1:{args.port}/v1', flush=True)
ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
