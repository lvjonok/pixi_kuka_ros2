"""scripts/ is not a package: put it on the path, as crisp_session.py does for kuka_crisp."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
