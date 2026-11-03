"""Noise diagnostics retain bad noise and reject discontinuous/simulated data."""
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from studio_data import FIELDS,load_run,summarize,summary_text


def rows(count=151):
    result=[]
    for i in range(count):
        r=dict.fromkeys(FIELDS,0)
        r.update(ms=i*20,state=0,flags=4,u=0,limit=150,adc=740+i%2,
                 zero=740,encoder=17,theta=math.radians(1 if i%2 else -1),
                 dtheta=3.0 if i%2 else -3.0)
        result.append(r)
    return result


class NoiseSummary(unittest.TestCase):
    def test_noisy_velocity_is_not_an_exclusion_threshold(self):
        s=summarize(rows(150));n=s['stationary_noise_candidate']
        self.assertEqual(n['frames'],150);self.assertEqual(n['duration_s'],2.98)
        self.assertEqual((n['adc_min'],n['adc_max']),(740,741))
        self.assertAlmostEqual(n['theta_std_deg'],1)
        self.assertAlmostEqual(n['dtheta_rms_rad_s'],3)
        self.assertEqual(n['dtheta_peak_rad_s'],3)
        self.assertFalse(n['physical_stillness_confirmed'])
        self.assertIn('候选静止段',summary_text(s));self.assertIn('不等于摆杆物理静止',summary_text(s))

    def test_two_second_boundary(self):
        self.assertIsNone(summarize(rows(100))['stationary_noise_candidate'])
        self.assertEqual(summarize(rows(101))['stationary_noise_candidate']['duration_s'],2)

    def test_encoder_and_zero_changes_split(self):
        for key in ('encoder','zero'):
            with self.subTest(field=key):
                rr=rows()
                for r in rr[75:]:r[key]+=1
                self.assertIsNone(summarize(rr)['stationary_noise_candidate'])

    def test_gap_or_reset_cannot_join_segments(self):
        for shift in (100,-1500):
            with self.subTest(shift=shift):
                rr=rows()
                for r in rr[75:]:r['ms']+=shift
                self.assertIsNone(summarize(rr)['stationary_noise_candidate'])

    def test_invalid_status_splits(self):
        for field,value in [('state',3),('flags',5),('flags',0),('flags',516),('u',1)]:
            with self.subTest(field=field,value=value):
                rr=rows();rr[75][field]=value
                self.assertIsNone(summarize(rr)['stationary_noise_candidate'])

    def test_longest_segment_and_missing_historical_fields(self):
        rr=rows(260);rr[110]['u']=1
        n=summarize(rr)['stationary_noise_candidate']
        self.assertEqual(n['start_index'],111);self.assertEqual(n['end_index'],260)
        for r in rr:r.pop('encoder')
        self.assertIsNone(summarize(rr)['stationary_noise_candidate'])

    def test_natural_u32_timer_rollover_is_continuous(self):
        rr=rows()
        for r in rr:r['ms']=(r['ms']+2**32-1000)%(2**32)
        self.assertEqual(summarize(rr)['stationary_noise_candidate']['duration_s'],3)

    def test_simulated_rows_do_not_get_stationary_statistics(self):
        s=summarize(rows(),is_simulation=True)
        self.assertIsNone(s['stationary_noise_candidate'])
        self.assertIn('不计算实物候选静止段',summary_text(s))
        self.assertNotIn('板端角速度 RMS',summary_text(s))

    def test_old_csv_and_both_simulation_metadata_conventions(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'old.csv'
            with p.open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader();w.writerows(rows())
            self.assertIsNotNone(load_run(p)['summary']['stationary_noise_candidate'])
            for meta in ({'is_simulation':True},{'kind':'rtl_nonlinear_simulation'}):
                p.with_suffix('.meta.json').write_text(json.dumps(meta))
                s=load_run(p)['summary']
                self.assertIsNone(s['stationary_noise_candidate']);self.assertTrue(s['is_simulation'])
                self.assertIn('不代表实物验收',summary_text(s))

    def test_empty_summary_remains_displayable(self):
        s=summarize([])
        self.assertIsNone(s['stationary_noise_candidate']);self.assertIn('0 帧',summary_text(s))


if __name__=='__main__':unittest.main(verbosity=2)
