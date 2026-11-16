import json
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'


def get_tg_input_devices(process_name: str, usbgpu: bool = False):
  with open(TG_INPUT_DEVICES_PATH) as f:
    return json.load(f)[process_name]['default' if not usbgpu else 'usbgpu']
