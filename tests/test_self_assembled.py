"""Project-owned procedures for assembled robots; all device operations mocked."""
from types import SimpleNamespace as NS
import pytest
from core.io import InputError,load,save
from programs.common import run_test
from programs.stage_02_kinematics import verify_joint_limits


@pytest.mark.parametrize('key',['operating_procedure','manufacturer_procedure'])
def test_project_procedure_and_legacy_field_preserve_execution_checks(demo,monkeypatch,key):
    state={'commands':[]}
    class MockOnly:
        capabilities={'command':True,'read_state':True,'stop':True,'watchdog':True}
        def read_state(self):return {'q:j1':0,'q:j2':0}
        def command(self,q):state['commands'].append(q)
        def stop(self):state['stop']=True
        def close(self):state['closed']=True
    monkeypatch.setattr(run_test,'load_adapter',lambda cfg:MockOnly())
    monkeypatch.setattr(run_test.time,'sleep',lambda seconds:None)
    interface={'class':'unit_test_mock','operating_procedure':None,key:'UNIT TEST PROJECT PLAN v1',
        'commissioning_reference':'UNIT TEST MOCK ONLY','limits_confirmed':True,'start_tolerance_rad_or_m':.001}
    save(demo/'project_interface.yaml',interface)
    args=NS(target='real',interface=demo/'project_interface.yaml',trajectory=demo/'replay.csv',urdf=demo/'nominal.urdf',execute=True,output_data=demo/'mock.csv')
    report=run_test.run(args)
    assert len(state['commands'])==10 and state['stop'] and state['closed']
    assert report['command']=='run-test'
    # The same configuration must still reject unconfirmed limits before opening an adapter.
    interface['limits_confirmed']=False;save(args.interface,interface)
    monkeypatch.setattr(run_test,'load_adapter',lambda cfg:pytest.fail('Adapter must not open'))
    with pytest.raises(InputError,match='limits'):run_test.run(args)


@pytest.mark.parametrize('procedure',[None,'',True])
def test_missing_project_procedure_blocks_before_device_access(demo,monkeypatch,procedure):
    save(demo/'project_interface.yaml',{'class':'unit_test_mock','operating_procedure':procedure,
        'commissioning_reference':'UNIT TEST','limits_confirmed':True})
    monkeypatch.setattr(run_test,'load_adapter',lambda cfg:pytest.fail('No device access permitted'))
    with pytest.raises(InputError,match='operating procedure'):
        run_test.run(NS(target='real',interface=demo/'project_interface.yaml',trajectory=demo/'replay.csv',urdf=demo/'nominal.urdf',execute=True,output_data=demo/'must_not_exist.csv'))


def test_project_operating_range_check_is_not_a_mechanical_stop_search(demo):
    meta=load(demo/'limits_manifest.yaml');meta.update(event_kind='operating_range_check',procedure_reference='SYNTHETIC project operating-range plan')
    save(demo/'limits_manifest.yaml',meta)
    spec=load(demo/'limits_spec.yaml')
    for v in spec['joints'].values():v.update(kind='operating_range_check',source='SYNTHETIC project range specification')
    save(demo/'limits_spec.yaml',spec)
    out=verify_joint_limits.run(NS(data=demo/'limits.csv',manifest=demo/'limits_manifest.yaml',specification=demo/'limits_spec.yaml'))
    assert out['status']=='synthetic_pass' and not out['writes_hardware_limits']
    assert out['mechanical_stops_are_not_operating_limits']
    meta['procedure_reference']=None;save(demo/'limits_manifest.yaml',meta)
    with pytest.raises(InputError,match='project/component'):
        verify_joint_limits.run(NS(data=demo/'limits.csv',manifest=demo/'limits_manifest.yaml',specification=demo/'limits_spec.yaml'))
