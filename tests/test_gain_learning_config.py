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
        return dict(outdoor_temperature_sensor='outside', other_units_operating=['other_a', 'other_b'])

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

    def test_automatic_mode_and_freeze(self):
        config = module.GAIN_LEARNING_SCHEMA(dict(self.base(), enabled=False))
        self.assertFalse(config['enabled'])
        self.assertNotIn('mode', config)
        self.assertNotIn('fireplace_active_sensor', config)
        self.assertNotIn('fireplace_state_valid_sensor', config)
        self.assertNotIn('fusion_wall_weight_sensor', config)

    def test_optional_inputs_are_explicit_sensor_references(self):
        config = module.GAIN_LEARNING_SCHEMA(dict(self.base(),
            fireplace_active_sensor='fireplace', fireplace_state_valid_sensor='rf_valid',
            fusion_wall_weight_sensor='wall_percent'))
        self.assertEqual(str(config['fireplace_active_sensor']), 'fireplace')
        self.assertEqual(str(config['fireplace_state_valid_sensor']), 'rf_valid')
        self.assertEqual(str(config['fusion_wall_weight_sensor']), 'wall_percent')
        with self.assertRaises(Exception):
            module.GAIN_LEARNING_SCHEMA(dict(self.base(), fireplace_state_valid_sensor='rf_valid'))

    def test_experimental_v1_config_migrates(self):
        with self.assertLogs(level='WARNING'):
            config = module.GAIN_LEARNING_SCHEMA(dict(outdoor_temperature='outside',
                external_heat='fireplace', context_valid='rf_valid', fusion_weight='wall',
                mode='heat', other_units_operating=['other_a','other_b']))
        self.assertNotIn('mode', config)
        self.assertNotIn('external_heat', config)
        self.assertEqual(str(config['fireplace_active_sensor']), 'fireplace')

    def test_other_heads_required_and_native_weather_allowed(self):
        config = module.GAIN_LEARNING_SCHEMA(dict(other_units_operating=['other_a','other_b']))
        self.assertNotIn('outdoor_temperature_sensor', config)
        with self.assertRaises(Exception):
            module.GAIN_LEARNING_SCHEMA({})


if __name__ == '__main__':
    unittest.main()
