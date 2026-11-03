"""Field handover: units, sync, relocation, stale inputs and applied geometry."""
from pathlib import Path
from types import SimpleNamespace as NS
from zipfile import ZipFile
import shutil
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from core.cli import main
from core.io import InputError, load, save, sha256, csv_rows, write_csv, evidence_paths
from core.fieldwork import init_project, edit_parameter, relative_file
from core.field_validation import pair_poses, transform_poses, apply_kinematics, verify_applied, offset_feedback, apply_drives
from core.model import Model
from core.optimization import predict
from core.units_and_frames import transform, pose_from_row

ROOT=Path(__file__).resolve().parents[1]


def test_project_headers_and_edit_preserve_unconfirmed_sources(demo,tmp_path):
    archive=tmp_path/'vendor.zip'
    with ZipFile(archive,'w') as z:z.write(demo/'nominal.urdf','model/robot.urdf')
    p=tmp_path/'项目 有空格'
    init_project(NS(project=p,archive=archive,docx=None,step=None),ROOT)
    assert not Path(load(p/'project.yaml')['baseline']).is_absolute()
    assert (p/'parameters/现场测量与交付记录.xlsx').is_file()
    assert (p/'data/response_real.csv').read_text().startswith('t,joint,')
    assert 'q:j1' in (p/'data/kinematics_train.csv').read_text()
    assert load(p/'data/kinematics_train.yaml')['data_kind'] is None
    from core.schemas import parameters
    assert all(r['status']=='unconfirmed' for r in parameters(p/'parameters/robot_parameters.xlsx')['joints'])
    edit_parameter(NS(sheet='joints',name='j1',field='lower',value='-1.7',source='UNIT TEST ONLY'),p)
    assert len(list((p/'parameters/history').glob('*.xlsx')))==1
    with pytest.raises(InputError):relative_file(p,'../escape.csv')


@pytest.fixture
def pairing_case(demo):
    save(demo/'project.yaml',{'baseline':'nominal.urdf'})
    write_csv(demo/'feedback.csv',[{'sequence':i,'sample_time':float(i),'joint':n,'q':90 if n=='j1' else 0} for i in range(3) for n in ('j1','j2')])
    write_csv(demo/'poses.csv',[{'sample_id':str(i),'sample_time':i+.001,'x':1000,'y':0,'z':0,'qx':0,'qy':0,'qz':0,'qw':1,'quality':'valid'} for i in range(3)])
    meta={'data_kind':'synthetic','clock':'shared test clock','role':'calibration','source':'KNOWN ANSWER TEST',
        'dataset_id':'fixture','observed_frame':'base','pose_source':'synthetic oracle','reference_instrument':'SYNTHETIC','coordinates_id':'publisher'}
    for name in ('feedback','poses'):save(demo/(name+'.yaml'),{**meta,'sha256':sha256(demo/(name+'.csv'))})
    cfg={'feedback':'feedback.csv','poses':'poses.csv','feedback_manifest':'feedback.yaml','pose_manifest':'poses.yaml',
        'output_data':'paired.csv','output_manifest':'paired.yaml','role':'calibration','dataset_id':'new-synthetic','observed_frame':'base',
        'maximum_time_difference_s':.01,'pose_time_offset_s':0,'time_sync_reference':'test clock',
        'pose_length_unit':'mm','joint_angle_unit':'deg','joint_length_unit':'m','required_quality':'valid'}
    save(demo/'pairing.yaml',cfg)
    return demo


def test_pairing_declared_units_preserve_data_kind(pairing_case):
    p=pairing_case;pair_poses(p,p/'pairing.yaml')
    row=csv_rows(p/'paired.csv')[0]
    assert float(row['q:j1'])==pytest.approx(np.pi/2)
    assert float(row['x'])==pytest.approx(1)
    assert load(p/'paired.yaml')['data_kind']=='synthetic'


@pytest.mark.parametrize('damage',['time','quality','missing_joint','reused_frame','hash','quaternion'])
def test_pairing_rejects_bad_observations(pairing_case,damage):
    p=pairing_case
    if damage=='missing_joint':
        write_csv(p/'feedback.csv',csv_rows(p/'feedback.csv')[:-1])
        meta=load(p/'feedback.yaml');meta['sha256']=sha256(p/'feedback.csv');save(p/'feedback.yaml',meta)
    else:
        rows=csv_rows(p/'poses.csv')
        if damage=='time':rows[0]['sample_time']=100
        if damage=='quality':rows[0]['quality']='lost'
        if damage=='reused_frame':rows[1]['sample_time']=.002
        if damage=='quaternion':rows[0]['qw']=2
        if damage=='hash':rows[0]['x']=1001
        write_csv(p/'poses.csv',rows)
        if damage!='hash':
            meta=load(p/'poses.yaml');meta['sha256']=sha256(p/'poses.csv');save(p/'poses.yaml',meta)
    with pytest.raises(InputError):pair_poses(p,p/'pairing.yaml')
    assert not (p/'paired.csv').exists()


def test_two_marker_transform_known_rotations_and_translations(demo):
    mb=transform([.2,.4,.8],[.3,-.1,.5]);bb=transform([.05,-.02,.1],[.1,0,-.2])
    et=transform([.02,.03,.04],[0,.2,-.1]);expected=transform([.6,.1,.5],[.2,.4,.6])
    mt=mb@np.linalg.inv(bb)@expected@et
    for name,pose in [('base',mb),('tool',mt)]:
        q=Rotation.from_matrix(pose[:3,:3]).as_quat()
        write_csv(demo/(name+'.csv'),[{'sample_id':name,'sample_time':1,**dict(zip(('x','y','z'),pose[:3,3]*1000)),**dict(zip(('qx','qy','qz','qw'),q)),'quality':'valid'}])
        save(demo/(name+'.yaml'),{'dataset_id':name,'role':'calibration','data_kind':'synthetic','clock':'shared','observed_frame':'camera',
            'source':'TEST','pose_source':'known matrices','reference_instrument':'TEST','sha256':sha256(demo/(name+'.csv'))})
    cfg={'marker_poses':'tool.csv','marker_manifest':'tool.yaml','base_marker_poses':'base.csv','base_marker_manifest':'base.yaml',
        'base_to_base_marker_xyz_rpy':[.05,-.02,.1,.1,0,-.2],'tool_to_tool_marker_xyz_rpy':[.02,.03,.04,0,.2,-.1],
        'installation_reference':'synthetic survey','input_length_unit':'mm','maximum_time_difference_s':.01,'required_quality':'valid',
        'observed_frame':'robot_base','output_data':'converted.csv','output_manifest':'converted.yaml'}
    save(demo/'transform.yaml',cfg);transform_poses(demo,demo/'transform.yaml')
    np.testing.assert_allclose(pose_from_row(csv_rows(demo/'converted.csv')[0]),expected,atol=1e-12)
    assert load(demo/'converted.yaml')['data_kind']=='synthetic'


@pytest.fixture
def application_case(demo,tmp_path):
    p=tmp_path/'field project';(p/'parameters').mkdir(parents=True)
    shutil.copytree(ROOT/'configs',p/'configs');shutil.copytree(demo,p/'data')
    for source,target in [('parameters.xlsx','parameters/robot_parameters.xlsx'),('mechanisms.yaml','configs/mechanisms.yaml'),('acceptance.yaml','configs/acceptance.yaml')]:shutil.copy2(demo/source,p/target)
    save(p/'project.yaml',{'baseline':'data/nominal.urdf'})
    nominal=p/'models/nominal_R01/robot.urdf';r=p/'results/R01';r.mkdir(parents=True)
    def execute(args):assert main(list(map(str,args)))==0
    execute(['build-urdf','--parameters',p/'parameters/robot_parameters.xlsx','--baseline',p/'data/nominal.urdf','--mechanisms',p/'configs/mechanisms.yaml','--destination',nominal.parent,'--output',r/'nominal.json'])
    cfg=load(p/'data/calibration.yaml')
    cfg['free_parameters'] += [{'kind':'joint_origin','joint':'j2','component':'x','lower':-.1,'upper':.1},
        {'kind':'tool','component':'y','lower':-.1,'upper':.1},{'kind':'base','component':'z','lower':-.1,'upper':.1}]
    save(p/'data/calibration.yaml',cfg);model=Model(nominal)
    for role in ('calibration','validation'):
        path=p/f'data/kinematics_{role}.csv';rows=csv_rows(path)
        for row in rows:
            pose=predict(model,row,cfg,cfg['free_parameters'],[.025,-.035,.012,.008,-.004])
            row.update(dict(zip(('x','y','z'),pose[:3,3])));row.update(dict(zip(('qx','qy','qz','qw'),Rotation.from_matrix(pose[:3,:3]).as_quat())))
        write_csv(path,rows);meta=load(path.with_suffix('.yaml'));meta.update(sha256=sha256(path),coordinates_id='test-publisher');save(path.with_suffix('.yaml'),meta)
    execute(['calibrate-kinematics','--data',p/'data/kinematics_calibration.csv','--manifest',p/'data/kinematics_calibration.yaml','--urdf',nominal,'--config',p/'data/calibration.yaml','--output',r/'fit.json'])
    execute(['validate-kinematics','--data',p/'data/kinematics_validation.csv','--manifest',p/'data/kinematics_validation.yaml','--urdf',nominal,'--calibration',r/'fit.json','--acceptance',p/'configs/acceptance.yaml','--output',r/'holdout.json'])
    return p


def test_apply_geometry_offsets_frames_once_and_relocate(application_case,tmp_path):
    p=application_case;apply_kinematics(p,'R01');assert verify_applied(p,'R01')==0
    out=load(p/'results/R01/verify_applied.json')
    assert out['metrics']['position_m']['max_abs']<1e-9
    assert out['before_metrics']['position_m']['max_abs']>.001
    assert out['application_equivalence']['passed'] and not out['is_real_robot_acceptance']
    moved=tmp_path/'搬走 目录';shutil.move(str(p),str(moved));assert verify_applied(moved,'R01')==0
    for path in (moved/'results/R01').glob('*.json'):
        report=load(path);assert report['input_path_base']=='report_directory';evidence_paths(report,path)


def test_changed_parameters_cannot_apply_old_fit(application_case):
    p=application_case;edit_parameter(NS(sheet='joints',name='j1',field='lower',value='-1.7',source='TEST CHANGE'),p)
    with pytest.raises(InputError):apply_kinematics(p,'R01')
    assert not (p/'models/calibrated_R01').exists()


def test_offset_feedback_checks_coordinate_definition_and_double_application(application_case):
    p=application_case;apply_kinematics(p,'R01');write_csv(p/'data/state.csv',[{'joint':'j1','q':.2},{'joint':'j2','q':-.1}])
    save(p/'data/state.yaml',{'dataset_id':'fixture','role':'validation','data_kind':'synthetic','units':{'length':'m','angle':'rad','time':'s','effort':'N_or_Nm'},
        'clock':'shared','source':'TEST','coordinates_id':'test-publisher','sha256':sha256(p/'data/state.csv')})
    cfg={'data':'data/state.csv','manifest':'data/state.yaml','fit_input_coordinates_id':'test-publisher','output_data':'data/offset.csv','output_manifest':'data/offset.yaml'}
    save(p/'configs/offset.yaml',cfg);offset_feedback(p,'R01',p/'configs/offset.yaml')
    assert float(csv_rows(p/'data/offset.csv')[0]['q'])==pytest.approx(.225)
    cfg.update(data='data/offset.csv',manifest='data/offset.yaml',output_data='data/double.csv',output_manifest='data/double.yaml');save(p/'configs/offset.yaml',cfg)
    with pytest.raises(InputError):offset_feedback(p,'R01',p/'configs/offset.yaml')
    assert not (p/'data/double.csv').exists()


def test_candidate_pd_application_needs_effort_limits(application_case):
    p=application_case;apply_kinematics(p,'R01')
    assert main(list(map(str,['tune-drives','--data',p/'data/drives.csv','--manifest',p/'data/drives.yaml','--output',p/'results/R01/drives-fit.json'])))==0
    with pytest.raises(InputError,match='max_effort'):apply_drives(p,'R01')
    path=p/'models/calibrated_R01/isaac_applied.yaml';cfg=load(path);cfg['drives']={'j1':{'max_effort':50},'j2':{'max_effort':50}};save(path,cfg)
    apply_drives(p,'R01');assert load(path.with_name('isaac_drives_candidate.yaml'))['drives']['j1']['stiffness']==pytest.approx(100)
