"""Verify the official checkpoints already versioned in this fork."""
import hashlib
from pathlib import Path

FOLDER=Path(__file__).parents[1]/'tiny-pulp-dronet-v3/model'
CHECKSUMS={
    'pulp-dronet-v3-resblock-1.0.pth':'943ab4cbbc28cd711e874559dcb3ba0419a0ae1250dc8c94665f4a46d878ff6e',
    'tiny-pulp-dronet-v3-dw-pw-0.125.pth':'9775bb595cbc8dba4043b95a31dbec2674789522d339913e67c9655cf5d84729',
}

if __name__=='__main__':
    for filename,expected in CHECKSUMS.items():
        actual=hashlib.sha256((FOLDER/filename).read_bytes()).hexdigest()
        if actual!=expected:raise SystemExit('Official checkpoint checksum mismatch: '+filename)
        print(f'{filename}: {actual}')
