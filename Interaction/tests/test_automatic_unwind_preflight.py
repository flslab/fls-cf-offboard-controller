"""Automatic unwind must be an explicit, freshly confirmed firmware contract."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from Interaction.mission_profiles import resolve_mission_profiles
from Interaction.firmware_image_contract import mission_parameters
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters
from Interaction.tests import test_scurve_firmware_preflight as scurve
from Interaction.tests import test_firmware_auto_brake_preflight as baseline

class AutomaticUnwindPreflightTests(unittest.TestCase):
    def fixture(self):
        mission=resolve_mission_profiles({'Interaction':{'config':{'behavior':'level_coast','duration':60,
            'wrench_interaction_profile':'level_coast','level_coast':{'coast_command_mode':'scurve'},
            'wrench_interaction':{'firmware_auto_brake':{'analytic_profile':{
                'auto_unwind':True,'end_velocity_m_s':0,'position_tracking_bandwidth':0,
                'marker_from_imu_m':[0,0,.086]}}}}}})
        mode=mission['Interaction']['config']['wrench_interaction']['firmware_auto_brake']
        c=baseline.FirmwareAutoBrakePreflightTests().controller(mode);c.mission=mission;c.args.controller_type="pid";c.prepare_firmware_auto_brake()
        p=scurve.SCurvePreflightTests().velocity_params()
        for key,value in c._firmware_analytic_expected.items():
            group,name=key.split('.');p.toc.toc.setdefault(group,{})[name]=None;p.replies[key]=str(value)
        for key,value in {'hlCommander.pRelEnd':1,'hlCommander.pRelAMax':8.23,'hlCommander.pRelTMax':40,
                          'hlCommander.pRelHoldG':0}.items():
            g,n=key.split('.');p.toc.toc.setdefault(g,{})[n]=None;p.replies[key]=str(value)
        p.replies.update({'pRelResp.runtime':'1','pRelResp.id':'123','pid_rate.roll_kp':'90'});p.set_value.side_effect=lambda k,v:p.replies.update({k:v})
        c.cf=SimpleNamespace(param=p);c.cfg=SimpleNamespace(PID_VALUES={});c.use_flowdeck=False;c.args.drone_id='fixture'
        return c,p

    def setup(self,c):
        with patch('Interaction.firmware_response_model.confirm_pid_context',return_value={'pid_rate.roll_kp':90}), \
             patch('Interaction.firmware_response_model.load_model',return_value={'source_log':'fixture'}), \
             patch('Interaction.firmware_response_model.upload_model',return_value={'pRelResp.id':123}):
            c._setup_firmware_auto_brake_params()

    def test_missing_api_rejects_before_writes_and_readonly_capability_is_never_written(self):
        c,p=self.fixture();del p.toc.toc['hlCommander']['pRelUnwVer']
        with self.assertRaisesRegex(RuntimeError,'pRelUnwVer'):self.setup(c)
        p.set_value.assert_not_called()
        p.toc.toc['hlCommander']['pRelUnwVer']=None;self.setup(c)
        self.assertNotIn('hlCommander.pRelUnwVer',[x.args[0] for x in p.set_value.call_args_list])
        self.assertIn('hlCommander.pRelUnwVer',p.requested)
        self.assertEqual(c._firmware_curve_expected['hlCommander.pRelUnwAuto'],1)
        self.assertEqual(c._firmware_curve_capabilities['hlCommander.pRelUnwVer'],1)
        self.assertEqual(p.set_value.call_args_list[-1].args,('hlCommander.pRelAuto','1'))

    def test_wrong_capability_cannot_enable_and_is_rechecked_before_arm(self):
        c,p=self.fixture();self.setup(c);p.set_value.reset_mock();p.replies['hlCommander.pRelUnwVer']='0'
        def quick(param,*,expected):return confirm_firmware_mode_parameters(param,expected=expected,timeout_s=.001)
        with patch('Interaction.firmware_parameter_confirmation.confirm_firmware_mode_parameters',side_effect=quick):
            with self.assertRaisesRegex(RuntimeError,'pRelUnwVer'):self.setup(c)
            self.assertNotIn(('hlCommander.pRelAuto','1'),[x.args for x in p.set_value.call_args_list])
            with self.assertRaisesRegex(RuntimeError,'pRelUnwVer'):c.verify_firmware_auto_brake_ready()

    def test_no_coast_does_not_require_the_new_apis(self):
        c,p=self.fixture();c.mission['Interaction']['config']['level_coast']['coasting_enabled']=False
        c.prepare_firmware_auto_brake()
        self.assertFalse(any(k.startswith('mocapMarker.') or 'pRelUnw' in k or k.endswith('pRelEndV')
                             for k in c._firmware_analytic_expected))
        self.assertFalse(any(k.startswith('mocapMarker.') or 'pRelUnw' in k or k.endswith('pRelEndV')
                             for k in mission_parameters(c.mission)))
