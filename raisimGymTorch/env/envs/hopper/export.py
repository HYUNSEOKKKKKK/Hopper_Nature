"""Export actor/estimator parameters and matching observation statistics."""

import argparse
from pathlib import Path
import shutil
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from raisimGymTorch.helper.hopper_config import checkpoint_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-w', '--weight', type=checkpoint_path, required=True)
    parser.add_argument('--output', type=Path, help='default: checkpoint directory/network_<iteration>')
    args = parser.parse_args()
    import torch

    iteration = args.weight.stem.split('_')[1]
    output = args.output or args.weight.parent / f'network_{iteration}'
    output.mkdir(parents=True, exist_ok=True)
    checkpoint = torch.load(args.weight, map_location='cpu', weights_only=True)
    for name in ('actor', 'estimator'):
        values = []
        for key, tensor in checkpoint[f'{name}_architecture_state_dict'].items():
            if key.startswith('architecture'):
                if tensor.ndim == 2:
                    tensor = tensor.transpose(0, 1)
                values.extend(str(value.item()) for value in tensor.flatten())
        (output / f'{name}.txt').write_text(', '.join(values) + '\n')
    for prefix in ('mean', 'var'):
        source = args.weight.parent / f'{prefix}{iteration}.csv'
        target = output / source.name
        if source.resolve() != target.resolve():
            shutil.copy2(source, target)
    print('Exported policy to:', output.resolve())


if __name__ == '__main__':
    main()
