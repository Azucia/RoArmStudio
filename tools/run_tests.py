"""Run offline regressions with isolated settings and physical device access blocked."""
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


def main():
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root / 'source'))
    sys.path.insert(0, str(root / 'tools'))
    sys.path.insert(0, str(root / 'sdk'))
    from PySide6.QtCore import QSettings

    with tempfile.TemporaryDirectory() as settings:
        QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, settings)
        def isolated_settings(organization, application):
            return QSettings(QSettings.IniFormat, QSettings.UserScope, organization, application)
        with patch('app.QSettings', side_effect=isolated_settings), \
             patch('serial.Serial', side_effect=AssertionError('Real serial access prohibited')), \
             patch('serial.tools.list_ports.comports', return_value=[]), \
             patch('cv2.VideoCapture', side_effect=AssertionError('Real camera access prohibited')):
            if len(sys.argv) > 1:
                suite = unittest.defaultTestLoader.loadTestsFromNames(sys.argv[1:])
            else:
                suite = unittest.TestSuite()
                for folder in ('source', 'tools'):
                    suite.addTests(unittest.defaultTestLoader.discover(
                        str(root / folder), top_level_dir=str(root / folder)))
            result = unittest.TextTestRunner(verbosity=2).run(suite)
    return not result.wasSuccessful()


if __name__ == '__main__':
    sys.exit(main())
