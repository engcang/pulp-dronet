"""Verify the small official checkpoint already versioned in this fork."""
import hashlib
from pathlib import Path

WEIGHTS=Path(__file__).parents[1]/'tiny-pulp-dronet-v3/model/pulp-dronet-v3-resblock-1.0.pth'
SHA256='943ab4cbbc28cd711e874559dcb3ba0419a0ae1250dc8c94665f4a46d878ff6e'

if __name__=='__main__':
    actual=hashlib.sha256(WEIGHTS.read_bytes()).hexdigest()
    if actual!=SHA256:raise SystemExit('Official checkpoint checksum mismatch')
    print(f'Official float v3 checkpoint verified: {actual}')
