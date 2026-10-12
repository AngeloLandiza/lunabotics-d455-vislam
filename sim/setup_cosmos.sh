#!/usr/bin/env bash
# One-time setup for the Cosmos 3 Edge rover agent (run/sim/arena/cosmos.sh):
# a Python 3.12 env in ~/cosmos-env with PyTorch, Hugging Face transformers and torchao, and the
# official nvidia/Cosmos3-Edge weights in the Hugging Face cache (~/.cache/huggingface).
# About 12 GB of downloads (PyTorch ~4 GB, the reasoner + vision encoder ~7.7 GB).
#
#   sim/setup_cosmos.sh                     official weights (int4 / int8 / bf16 in sim/cosmos_server.py)
#   COSMOS_NF4=1 sim/setup_cosmos.sh        also the community 4-bit build (--quant nf4, 2.3 GB)
set -eo pipefail
ENV="${COSMOS_ENV:-$HOME/cosmos-env}"
UV="$(command -v uv || echo "$HOME/.local/bin/uv")"
[ -x "$UV" ] || { echo "uv not found: curl -LsSf https://astral.sh/uv/install.sh | sh" >&2; exit 1; }

[ -d "$ENV" ] || "$UV" venv "$ENV" --python 3.12
# transformers >= 5.16 has the cosmos3_edge model; torchao quantizes it to int4/int8 at load
# time; bitsandbytes loads the community 4-bit build; torchvision is needed by its processor.
VIRTUAL_ENV="$ENV" "$UV" pip install torch torchvision "transformers>=5.16" accelerate torchao bitsandbytes pillow
"$ENV/bin/python" - "${COSMOS_NF4:-0}" <<'EOF'
import sys
from huggingface_hub import snapshot_download
# Only the reasoner (image + text -> text): its language model lives in transformer/ (shared with
# the video generator), plus the vision encoder. Skip the VAE and the demo assets.
path = snapshot_download('nvidia/Cosmos3-Edge', allow_patterns=['*.json', '*.jinja', 'tokenizer*',
                                                                 'transformer/*', 'vision_encoder/*'])
print('model in', path)
if sys.argv[1] == '1':
    print('nf4 in', snapshot_download('ubr-physical-ai/Cosmos3-Edge-NF4-bnb'))
EOF
echo "Done. Run: run/sim/arena/cosmos.sh   (the first start compiles the model for ~1 min)"
