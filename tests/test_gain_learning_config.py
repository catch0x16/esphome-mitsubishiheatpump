"""Validate the opt-in schema; requires the repository's ESPHome dependency."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'components/mitsubishi_heatpump/climate.py'
spec = importlib.util.spec_from_file_location('gain_learning_climate', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConfigTests(unittest.TestCase):
    def base(self):
        return dict(outdoor_temperature='outside', external_heat='fireplace',
                    context_valid='context', other_units_operating=['other_a', 'other_b'])

    def test_defaults(self):
        config = module.GAIN_LEARNING_SCHEMA(self.base())
        self.assertEqual(config['confirmation_pairs'], 24)
        self.assertEqual(config['minimum_gains'], [0, 0, 0])
        self.assertEqual(config['period'].total_milliseconds, 7200000)
        self.assertEqual(config['max_command_difference'], .5)

    def test_rejects_invalid_evidence_and_bounds(self):
        cases = [dict(confirmation_pairs=2), dict(confirmation_pairs=64, max_pair_attempts=24),
                 dict(observable_time='3h'), dict(minimum_gains=[-1,0,0]),
                 dict(maximum_gains=[500,10,7200]), dict(maximum_gains=[1,2]),
                 dict(minimum_gains=[251,0,0]), dict(period='8d'),
                 dict(guard_relative=float('nan')), dict(minimum_gains=[float('nan'),0,0]),
                 dict(other_units_operating=[]), dict(other_units_operating=['one']),
                 dict(other_units_operating=['a','b','c'])]
        for extra in cases:
            with self.subTest(extra=extra), self.assertRaises(Exception):
                module.GAIN_LEARNING_SCHEMA(dict(self.base(), **extra))

    def test_explicit_freeze_and_cooling(self):
        config = module.GAIN_LEARNING_SCHEMA(dict(self.base(), enabled=False, mode='cool'))
        self.assertFalse(config['enabled'])
        self.assertEqual(config['mode'], 'cool')

    def test_requires_context(self):
        for key in self.base():
            config = self.base()
            del config[key]
            with self.subTest(missing=key), self.assertRaises(Exception):
                module.GAIN_LEARNING_SCHEMA(config)


if __name__ == '__main__':
    unittest.main()
