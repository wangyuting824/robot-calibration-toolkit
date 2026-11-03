"""Generate the offline operator manual and a complete, linked parameter appendix."""
from pathlib import Path
from html import escape as h
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from core.cli import COMMANDS, parser
from core.schemas import SHEETS
from core.fieldwork import jobs


def table(headers, rows):
    return '<div class="scroll"><table><tr>'+''.join('<th>'+h(str(x))+'</th>' for x in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+h('' if v is None else str(v))+'</td>' for v in r)+'</tr>' for r in rows)+'</table></div>'


def build():
    catalog=json.loads((ROOT/'docs/field_catalog.json').read_text(encoding='utf-8'))
    sections=[]
    def section(name,title,content):
        sections.append((name,title,content))
    section('start','1　先看这里：现场只需要按顺序做', '''
<p>这是一套离开 Codex 也能运行的项目软件。Ubuntu x86_64 离线包带独立 Python、科学计算、Excel 读写、MuJoCo 和 USD 检查依赖。软件可以搬到硬盘、不同账号和带中文/空格的目录。使用可写磁盘；若移动硬盘不允许执行程序，先复制到电脑本地目录。</p>
<ol><li>完整解压工具包，打开本文件；终端进入解压目录，执行下面的自检。</li><li>打开 projects/旭创现场项目/parameters 中两本 Excel，核对机构、填写参数与来源；configs 中填写阈值、设备和试验设置。</li><li>测量设备按适用的厂家手册校准、装夹和采集；原始文件放 data/raw。按本手册的列名导出 CSV，配套填写 YAML 来源记录。</li><li>先核对反馈与零位，再采集运动学标定集和另采的验证集；运行拟合、留出验证、apply、verify-applied。</li><li>质量惯性与机构确认后导入 Isaac；按真实输入做响应试验，另采验证数据。</li><li>生成报告；按 D01—D06 归档，并交接完整项目备份和工具包。</li></ol>
<pre>bash 启动工具包.sh doctor
bash 启动工具包.sh check-bundle
bash 启动工具包.sh self-test --destination /tmp/robotcal_selfcheck_01
bash 启动工具包.sh status --project projects/旭创现场项目</pre>
<p>self-test 全部使用合成数据，只验证软件链路；会检查修正模型和证据文件换路径后是否仍可用。输出目录必须为空。check-bundle 应在刚解压时运行；之后主动修改过的项目文件与交接清单不符属于预期。</p>
<p><b>当前并未完成旭创真机标定。</b>还需要厂家确认资料、实机接口、独立测量和验收阈值。没有这些数据时，工具会明确拒绝正式验收，不会自动填出“正确参数”。</p>''')
    section('scope','2　已有资料、原清单要求与六类交付', '''
<p>原始资料三份：机器人 URDF 建模资料确认清单.docx、robot_urdf_external_joints.zip、机器人总装（对外）.STEP。原始清单要求补齐“结构关系、轴线/零位、运行范围”。本会话另外约定独立运动学验证和 Isaac 动力学/接触对齐；这是扩展交付范围。</p>
<p>现有模型为 63 links / 62 joints（14 revolute、48 fixed）。双臂以外的头颈、腰腿、轮组和手指大多固定；±20°、0.5 rad/s、20 N·m 及质量惯性是测试默认值，不能作为真机参数。STEP 的 12 个顶层组件、62 个叶级实例、30 个候选组/30 条候选连接不等于实际自由度。</p>
<p>原清单涉及 C/A/B/W 分组、P000—P061 零件和 K 编号。A 为 STEP 负 X 侧/CAD L，B 为正 X 侧/CAD R；现场按实物和图纸确认，不能只按左右称呼。保留原零件名称、装配路径、型号、版本、日期及 STEP 一致性说明。原文提及的 STEP候选连杆与连接关系确认.xlsx 填回版尚未收到；新生成的结构对应表仅是替代记录，不能冒充原表。</p>
<p>原 DOCX 两张内嵌表仍为待填，未规定末端 mm/°、轨迹或动力学允许误差。configs/acceptance.yaml 的 null 需项目确认。希望越小越好不能替代一个可验收的阈值；指标应结合测量能力、姿态范围、负载、速度和工作空间约定。</p>
'''+table(catalog['sheets']['交付物映射'][0],catalog['sheets']['交付物映射'][1:])+'''
<p>OpenArm 仅验证程序与链路。历史卡尺数据已找到，但右夹爪记录 incomplete，留出开度误差约 35 mm，不能用于修正旭创或直接应用于 OpenArm。源码原有 93 项检查通过；本次新增现场入口和搬迁验证见随包测试报告。</p>''')
    section('official','3　官方依据是什么', '''
<p><b>这是依据官方接口、厂家资料和公开算法实现的项目工具包；不是 NVIDIA 发布或认证的通用整机标定系统。</b>官方文档规定模型/API、相机位姿求解、系统辨识和驱动行为。旭创本机的回零、限位、运动、力矩标定和负载规程仍须厂家提供；不得拿其它品牌的动作流程替代。</p>
<ul><li>NVIDIA：6.1 URDF Importer、SysID、Gain Tuner、PhysX/OpenUSD；见完整链接表 S06—S08、S20—S24。</li><li>OpenCV：相机内参、ChArUco 及 PnP，见 S01—S03。只提供算法依据，不能预先承诺实际毫米精度。</li><li>设备厂家：角度编码器、事件锁存、动捕、六维跟踪、ATI 测力设备等；应按选定型号的安装/校准/采集手册操作。</li><li>项目程序：SciPy 拟合、数据配对、RNEA、接触近似、应用与报告；来源及限制见 docs/sources.md、sources.yaml、third_party.md。ROS robot_calibration 是维护者发布的社区项目，不是旭创厂家的规程。</li></ul>
<p>六维位姿测 XYZ 与方向；六维力/力矩测 Fx/Fy/Fz/Mx/My/Mz。两者不能替代。卡尺的 0.01 mm 显示分辨率也不等于测量准确度。</p>''')
    section('tools','4　带什么工具，怎么用，采什么', '''
<p>下面每一类表示必须具备的能力，不表示都要购买。厂家可信资料、复用设备、租赁或计量服务均可。独立六维位姿路线选适合的一条；尚未选定设备时无法给出真实 SDK 接线和采样命令，CSV 路线可先使用厂家软件导出结果。</p>
'''+table(catalog['sheets']['工具准备清单'][0],catalog['sheets']['工具准备清单'][1:])+'''
<ol><li><b>结构和轴线：</b>核对原始结构表、CAD 和实物。只对确认可动关节标注轴上一点、单位方向、正向箭头、参考坐标和零姿态。原表数值为 mm/deg；程序表内用 m/rad。图纸无法满足精度时，选合适三维计量设备/服务，按其厂家规程安装和测量；软件不会从 STEP 自动发现真实轴线。</li>
<li><b>零位和反馈：</b>使用旭创适用的零位工装或独立输出端角度/位移参考；厂家软件读取原始反馈。按批准的工况记录多个位置的 raw/reference，明确电机端/输出端、比例、减速关系和正方向。标定与验证另采，覆盖有效范围；不得用同一原始反馈生成 reference 自证。程序至少需要 3 个不同位置，实际采多少由覆盖和测量能力决定。</li>
<li><b>限位：</b>先取得正常运行范围和厂家规程，区分软件边界、开关触发、机械边界记录。厂家允许的方式操作，由驱动/PLC 锁存触发瞬间位置，另记停止位置、方向与重复次数；必要时用独立角度/位移参考。没有开关则不虚构开关试验。本包只核验已采记录，不提供自动撞止挡/堵转/搜索极限程序。速度、加速度、额定/峰值力矩按厂家资料确认。</li>
<li><b>相机路线：</b>按 OpenCV 官方教程标定相机内参/畸变，测定板尺寸和安装关系，固定光学设置；将基座板与末端板刚性安装。采原始图像、两块板在相机坐标的位姿、识别质量和测量时间。先静态停稳采集；用独立尺寸/参考检验现场测量能力。相机重投影像素误差不是末端 mm 精度。</li>
<li><b>动捕路线：</b>按 OptiTrack 等厂家手册标定相机空间，定义基座/末端刚体标记组，确认刚体原点和安装变换；导出位姿、时间、遮挡/跟踪质量。六维跟踪路线需配匹配六维靶标与参考坐标，普通单球靶不能给出完整方向。</li>
<li><b>质量惯性：</b>核对每个真实刚体、附件和负载；可用校验过的秤核质量，用完整材料/BOM/CAD 质量属性或适用辨识方案取得质心/惯性。明确单位、参考点/轴，转到连杆质心惯性坐标。称重不能直接测出惯性，装机不可拆时不强制拆机。</li>
<li><b>驱动/动力学：</b>按厂家批准试验记录实际输入、控制模式、q/dq/可信 ddq、力矩语义、前馈/饱和/负载和时间。电流不是自动可信的输出端关节力矩；末端六维力也不能直接给全部关节驱动力矩。优先按 NVIDIA SysID 适用流程或已有真实记录选择可辨识参数。</li>
<li><b>接触：</b>按实际材料、表面和负载设计试验，用适用测力装置配位移/滑速测量；法向、切向、滑动和冲击各采匹配观测。ATI 等设备先按手册安装、去皮/校验、定义坐标。滑动数据不能辨识静摩擦，线性柔顺结果不能直接当 PhysX 任意参数。</li>
<li><b>同步：</b>保留设备的测量时间与电脑接收时间，说明触发/PTP/共同时钟或独立测得的时钟关系。采样行号不能当时间；不能把曲线任意平移来美化误差。停稳采集可降低动态同步误差，动态试验仍需实际同步依据。</li></ol>''')
    section('files','5　填到哪里：文件、列名、单位', '''
<pre>工具包/
  启动工具包.sh  field.py  run.py
  runtime/        独立Python与已安装依赖（Linux离线包）
  wheelhouse/     同一Python/平台的依赖安装文件
  docs/           本手册、来源、接口说明、完整附表
  projects/旭创现场项目/
    project.yaml                  型号、robot_id、基线和原资料哈希
    originals/                    三份原资料，不覆盖
    baseline/                     原始模型，仅作来源
    parameters/robot_parameters.xlsx    程序读取的参数主表
    parameters/现场测量与交付记录.xlsx    人填写的机构、设备、工况和交付记录
    configs/                      阈值、拟合范围、限位、同步、Isaac、job设置
    data/raw/                     原始导出、图像、仪器校准和厂家规程
    data/*.csv + *.yaml            规范数据和来源记录
    models/nominal_R01/            本版本名义模型
    models/calibrated_R01/         修正模型、offset、坐标和应用记录
    simulation/R01/               本版本Isaac资产
    results/R01/                  JSON、CSV误差和日志
    reports/R01/                  HTML汇总及未完成项
    delivery/                     D01—D06归档设置</pre>
<p>Excel 里向量写空格分隔，如 <code>0 0 1</code>。程序主表统一 m/rad/kg/s/N/N·m，四元数 qx qy qz qw；URDF 的 RPY 是固定轴 XYZ。mm 除以 1000，deg 乘 π/180。惯性为 kg·m²，顺序 <code>ixx iyy izz ixy ixz iyz</code>；必须是关于质心、指定惯性轴的张量，不能直接复制关于 CAD 世界原点的量或混用积惯量符号。</p>
<p><b>robot_parameters.xlsx 的 9 张表：</b>robot_info 填型号/版本及单位；links 填名称、质量、COM、惯性、来源和确认状态；joints 填父子、类型、origin/axis、范围/速度/力矩、阻尼/摩擦和 mimic；geometry 填几何来源与安装变换；frames 记录基座/工具坐标；actuators 记录反馈与控制定义；mechanisms 记录联动/闭环；contacts 记录接触定义；sources 记录图纸、文件、修订和链接。各表完整列名见附录，缺项保留空白/unconfirmed，确认后填 confirmed 和真实来源。frames/actuators 的记录不会自动写入实机控制器。</p>
<p>初始主表从测试 URDF 重建，所有参数保持 unconfirmed。不要批量改 confirmed。头颈、腰腿、轮组、夹爪真实自由度、闭环及非线性联动需要先改结构；线性 mimic 只能描述实际线性联动。每个闭环要提供工程确认的约束实现，不能靠空列表认定无闭环。</p>
<p><b>configs/calibration.yaml：</b>observed_frame 为测量坐标名；tip_link 为要验证的模型末端；base_xyz_rpy = T_测量坐标_模型根；tool_xyz_rpy = T_模型末端_被测对象。先填已测安装关系，再按可观测性选择 free_parameters。每项 kind 是 joint_offset/base/tool/joint_origin；关节填 joint，坐标分量填 component，lower/upper 是允许修正范围且包含 0；position_sigma_m/angle_sigma_rad 用测量能力依据填写，不能为了降残差任意改权重。</p>
<p>例如只释放一个关节零偏的配置形状如下。<b>这只是字段形状，数值需按本机依据填写，不能原样当旭创参数：</b></p>
<pre>free_parameters:
  - kind: joint_offset
    joint: 实际关节名
    lower: 允许最小修正_rad或m
    upper: 允许最大修正_rad或m</pre>
<p><b>configs/limits.yaml：</b>joints.关节名 下填 lower/upper（rad 或 m）、kind（software_boundary/switch_latch/manufacturer_stop_record）、source、tolerance、repeatability_peak_to_peak、minimum_repeats。此表应对应所核验边界；URDF 的 lower/upper 填厂家确认的正常运行范围，两者不一定相同。连续旋转关节在主表标 continuous，不编造有限上下限。</p>
<p><b>configs/acceptance.yaml：</b>kinematics.max_position_m / max_orientation_rad 是真机位姿误差；asset 的六项是导入一致性容差；runtime_kinematics.max_position_m / max_orientation_rad 是仿真软件一致性容差，需自行增加并声明；dynamics.required_metrics 与 max_abs 声明 q/dq/tau/normal_force/slip_velocity 的验收范围。每种容差有不同用途。</p>
<p><b>configs/jobs.yaml：</b>统一入口使用的实际程序参数。reference-fit.options.f_scale 用参考测量单位填写稳健拟合尺度；所有路径相对项目目录。R01 是一组固定模型/配置/数据版本；更改参数或输入后用 R02，并将模型哈希、数据版本等一并更新重新验证。若只做后续一步且仍引用旧模型，应明确修改 job 的输入路径，不能借新版本名掩盖旧模型。</p>
'''+table(['数据文件','必须的CSV列','YAML另外填写'],[
['joint_reference_train / validation','sample_id,joint,raw,reference','role=calibration/validation；reference_kind、reference_definition、reference_instrument、raw_unit、reference_unit、robot_id、feedback_coordinates_id、urdf_sha256；SI角度rad/长度m'],
['limits','joint,boundary,angle；另留 direction,repeat,event_time,stop_position','role=validation；event_kind、procedure_reference；angle在直线关节中仍表示m'],
['joint_states','sequence,sample_time,joint,q,dq,effort','coordinates_id、joint_units、robot_id；q必须有效，未知dq/effort空白，不能补0'],
['poses / raw靶标','sample_id,sample_time,x,y,z,qx,qy,qz,qw,quality','observed_frame、pose_source、reference_instrument；质量标志与时钟依据'],
['kinematics_train / validation','sample_id,所有q:独立关节,x,y,z,qx,qy,qz,qw','role=calibration/validation，observed_frame、pose_source、coordinates_id；两组sample_id和dataset_id不同，另采'],
['dynamics_train','每个独立关节的q:名称,dq:名称,ddq:名称,tau:名称','torque_semantics=joint_actuator_effort、effort_reference；config.fixed_base=true、inertia_source、friction_velocity_scale'],
['drives_train','joint,q,dq,q_command,dq_command,pd_effort,saturated','torque_semantics=isolated_pd_effort，drive_type=force；总驱动力矩不能直接作PD分量'],
['contact_train','sliding: normal_force,slip_velocity,tangential_effort；normal: penetration,closing_velocity,normal_force；restitution: approach_speed,separation_speed','procedure_reference、force_reference；configs/contact.yaml选择匹配model；滑动另填velocity_scale'],
['response_real / sim','t,joint,所需q/dq/tau/normal_force/slip_velocity','role=validation；test_case_id,input_sha256,initial_state,payload_id,time_basis=test_relative两侧一致，另记录控制模式；tau需要双方effort_reference']])+'''
<p>每个 CSV 同名 YAML 至少填 dataset_id、role、data_kind、units、clock、source、sha256。data_kind 必须如实为 real/simulation/synthetic；units 固定为 {length: m, angle: rad, time: s, effort: N_or_Nm}，原始 mm/deg 数据的单位另外显式记录并先转换。相机或仪器厂家输出列名不同，应先按本表转换，保留原件和转换记录。</p>
<pre>bash 启动工具包.sh seal --project projects/旭创现场项目 --data data/kinematics_train.csv --manifest data/kinematics_train.yaml</pre>
<p>seal 只计算哈希并封存已填来源的数据，不会自动补来源。已经封存后不要覆盖原件；另存新文件、新 dataset_id 和 manifest。</p>''')
    section('collect','6　采集、坐标变换与时间配对', '''
<p><b>可直接走厂家导出路线：</b>机器人调试软件导出关节记录，测量仪器软件导出位姿/力数据；原始文件保存 data/raw；按上表列名生成 CSV 和来源 YAML。这条路线不依赖 Codex，也不要求设备一定有 Python SDK。自动在线采集需要在 adapters/ 中实现并验证该型号的 SDK；robot_adapter_template.py 和 metrology_adapter_template.py 只是接口模板，尚不是旭创驱动。ROS 只读、OpenArm 和 replay 成功不能替代旭创设备验收。</p>
<p>两块靶标都在同一测量坐标 M、相同时钟下：B 为机器人基座，Bm 为基座靶，E 为被测工具，Em 为末端靶。按工装测得 T_B_Bm、T_E_Em，仪器测 T_M_Bm、T_M_Em，则：</p>
<pre>T_B_E = T_B_Bm × inverse(T_M_Bm) × T_M_Em × inverse(T_E_Em)</pre>
<p>程序 transform-poses 实现此式。填写 configs/pose_transform.yaml：两份靶标 CSV/manifest、两项安装变换（xyz/rpy，m/rad）、installation_reference、input_length_unit、maximum_time_difference_s、required_quality 和 observed_frame。必须有同步的测量时间和合格质量。输出 poses.csv 为基座中工具的独立位姿，m、xyzw；它不会使用 URDF 生成真值。若设备直接输出所需坐标下的工具位姿，可跳过此步。所有变换都代表“子坐标在父坐标中的位姿”，不是反过来。</p>
<pre>bash 启动工具包.sh transform-poses --project projects/旭创现场项目</pre>
<p>然后填写 configs/pairing.yaml：feedback/poses 及对应 manifest、dataset_id、role、observed_frame、maximum_time_difference_s、pose_time_offset_s、time_sync_reference、三种输入单位和 required_quality。若更改过真实自由度，增加 urdf 指向该版本名义模型。两个时间轴的已知关系是“pose时间 + pose_time_offset_s = 反馈时钟”；偏移量来自独立同步依据，不从误差曲线反推。原始时间统一秒。</p>
<pre>bash 启动工具包.sh pair-poses --project projects/旭创现场项目</pre>
<p>pair-poses 按时间最近的完整关节帧配对，不插值、不自动删除坏点、不补缺关节、不裁限位。同一反馈帧重复用于不同位姿、任何关节时间超差、四元数不规范或质量失败均会拒绝。输出宽表、同名 manifest 和 pairing_audit.csv。验证集另采、换输出文件名、dataset_id 和 role，再单独配对。静态采集仍需记录停稳和测量时间，程序不自动判定厂家允许的运动状态。</p>
<p>若测量姿态仍是板而不是工具，calibration.yaml 的 tool_xyz_rpy 应为模型末端到板；若已换算到工具，则填模型末端到工具。不能在测量侧变换一次后，又在 tool_xyz_rpy 重复施加同一安装变换。</p>''')
    section('run','7　按顺序运行：从参数到URDF复验', '''
<p>下列命令均在工具包根目录执行；随包旭创项目已初始化。新机器人可用 init --project 新项目目录 --archive 厂家ZIP路径，并按需加 --docx、--step；生成的参数仍待确认。不同机器人共用求解与报告程序，真实自由度、列名、厂家接口、机构约束及测量布置按型号配置。</p>
<pre>bash 启动工具包.sh run --project projects/旭创现场项目 structure
bash 启动工具包.sh run --project projects/旭创现场项目 nominal
bash 启动工具包.sh run --project projects/旭创现场项目 reference-fit
bash 启动工具包.sh run --project projects/旭创现场项目 reference-check
bash 启动工具包.sh run --project projects/旭创现场项目 feedback
bash 启动工具包.sh run --project projects/旭创现场项目 limits</pre>
<p>reference-check 只有独立参考验证通过才导出反馈映射，并绑定具体 URDF、robot_id、坐标定义及证据哈希。feedback 只转换记录，q_model = scale×raw+offset；不改真机、不自动变换力矩、不外推。映射和 reference 数据需要 rad/m；若原始计数或电角度，先按有依据的传动定义转成 SI 并记录转换。实机正确坐标已由厂家标定证明时，可按批准依据直接导入已规范的 q，记录使用范围及理由。</p>
<p>独立运动学标定与验证的数据应使用同一份已定义的反馈坐标，manifest 填 coordinates_id。配置确认后：</p>
<pre>bash 启动工具包.sh run --project projects/旭创现场项目 fit
bash 启动工具包.sh run --project projects/旭创现场项目 holdout
bash 启动工具包.sh apply --project projects/旭创现场项目
bash 启动工具包.sh verify-applied --project projects/旭创现场项目</pre>
<p>先看 fit 的 identifiable、parameters_at_bound 和来源；秩不足需要固定耦合参数或调整覆盖后重采。holdout 用未参与拟合的数据，按约定位置/方向阈值判定。apply 会重新构建名义模型，发现主表已经变化则拒绝套用旧拟合。每个 job 保留 JSON 和日志；同一版本已有结果时不覆盖。</p>
<p>verify-applied 同时做两类比较：①新 URDF＋单独零偏＋坐标文件是否与拟合目标一致（数值检查，1e-9，仅软件容差）；②这套实际交付文件对独立测量的误差是否仍达标，给出改前/改后结果。它不会重新拟合，也不会把仿真数值误差称为真机精度。</p>
<p><b>修改某参数：</b>可直接编辑主表并保留原版本，也可用 edit 自动备份。示例参数名需替换为真实关节，数值必须来自实测：</p>
<pre>bash 启动工具包.sh edit --project projects/旭创现场项目 --sheet joints --name 实际关节名 --field lower --value '-1.2' --source '厂家文件版本和页码'
bash 启动工具包.sh run --project projects/旭创现场项目 --revision R02 structure
bash 启动工具包.sh run --project projects/旭创现场项目 --revision R02 nominal</pre>
<p>edit 会备份到 parameters/history，将相关状态恢复为 unconfirmed；审核后再确认。不要直接改生成 URDF 的数字后宣称标定完成，下一次重建会覆盖这种手工改动，也会破坏模型哈希绑定。</p>''')
    section('apply','8　标定结果究竟改在哪里，如何避免重复', table(['结果/测量','修改文件或字段','应用程序','必须复验'],[
['机构、轴线、范围、联动','参数主表joints与geometry；mechanisms.yaml；保留K/P/装配映射','structure → nominal；改变拓扑要重采/重拟合受影响链','结构、轴/锚点、mimic/闭环、范围及新版本模型'],
['反馈比例、方向、零位','nominal_R01/feedback_mapping.yaml（验证程序导出，不手改系数/哈希）','reference-fit → reference-check → feedback','独立关节参考；模型/控制器配置变更后重新验证映射'],
['选定joint_origin几何修正','calibrated_R01/robot.urdf 内 joint origin','apply自动烘焙原点修正','verify-applied；重新导入USD并复验'],
['运动学附加关节零偏','calibrated_R01/joint_offsets.yaml，q_model=q_拟合输入+offset','offset-feedback 转换只读长表；在线控制适配器需另验逆映射','verify-applied；不能同时再烘焙进joint origin'],
['基座/工具变换','scene_frames.yaml、isaac_applied.yaml 的 base_xyz_rpy/tool_frame','apply导出；Isaac导入器按配置创建场景挂载','verify-applied；实测坐标/工具安装核对'],
['质量、COM、惯性','links.mass/com_xyz/com_rpy/inertia与独立inertial_status/source','inertia → 重建URDF → 重导Isaac','质量惯性静态检查、匹配负载动态试验'],
['候选力驱动增益','isaac_drives_candidate.yaml → drives.关节.{type,stiffness,damping,max_effort}','drives-fit → apply-drives；Isaac角度增益转换由导入器做一次','匹配输入的独立response，不能复制厂家PID直接当PhysX增益'],
['材料摩擦/恢复','configs/isaac.yaml 或版本配置中 materials[]：path,static_friction,dynamic_friction,restitution,collision_paths','工程审核匹配物理模型后填写，重导资产','对应材料/表面/负载的独立接触试验'],
['RNEA附加惯量/摩擦/偏置、线性接触柔顺、延迟','辨识报告 + 明确适用的执行器/接触/延迟实现与配置','本包不自动将不同模型系数映射到PhysX；需明确同一物理模型的适配实现','存在未实现参数应列待完成，不能填相似字段冒充已应用']])+'''
<p>offset-feedback 使用 configs/offset_feedback.yaml，将规范反馈数据的 coordinates_id 与拟合训练集一致性检查后再加 joint_offsets；保留 q_fit_input 原列，拒绝重复、超范围和来源变化。输出仅用于只读回放/数据整理，不向硬件下发指令。</p>
<pre>bash 启动工具包.sh offset-feedback --project projects/旭创现场项目</pre>
<p>反馈映射仍绑定其原验证模型；重建后不能手改模型哈希“重新绑定”。可先按原映射获得拟合输入 q，再加本次运动学 offset；或对最终模型重新做独立参考验证。命令方向的逆关系应由厂家控制适配器实现并验证：q_拟合输入=q_model−offset，raw=(q_拟合输入−反馈零位偏置)/scale。本包现场入口不自动写控制器、更新硬件限位或使能电机。</p>
<p>URDF 不标准化表达完整执行器模型、控制延迟、接触材质和所有闭环；这些内容在 D03 场景/配置与适配器中交付。未实现的物理模型不能因为 URDF 能打开就标为完成。</p>''')
    section('sim','9　Isaac与MuJoCo怎么运行，动力学怎么复验', '''
<p><b>Isaac另装：</b>离线工具包没有捆绑 NVIDIA GPU 驱动、Isaac Sim 21GB级SDK、ROS 或厂家 SDK。实际电脑需先按 NVIDIA 官方 6.1 要求安装并启动验证。6.1 pip版要求 Python3.12、Linux GLIBC2.35+；不要把本包科学计算环境覆盖进 SDK 的已验证依赖环境。当前电脑曾验证 6.1.0-rc.26，不能据此宣称未知园区电脑或所有6.1发行版已经验证。</p>
<p>本机配置单独写在项目 <code>.machine.json</code>，例 <code>{"isaac_python":"/实际安装位置/python.sh"}</code>，pip环境则填该环境bin/python。只有这份本机设置允许绝对路径；搬电脑后只改它。额外 openpyxl 等依赖用该 SDK 的 Python 安装；可从 wheelhouse 离线安装 openpyxl/PyYAML/et-xmlfile，勿无差别升级 SDK 的 NumPy/SciPy。Isaac 许可由实际用户按官方要求接受，本程序不替用户接受许可。</p>
<p>正式离线去园区前，应携带匹配版本的 NVIDIA 安装包/容器镜像及预缓存扩展/所用本地资产、匹配显卡驱动与厂家SDK；先在目标电脑断网启动同一项目验证。官方资产和部分扩展可能联网，不能仅有一个pip命令就声称完全离线。采集电脑与仿真工作站可分开，通过整个项目备份交换数据。</p>
<p>Isaac job 默认读取 <code>models/calibrated_{revision}/isaac_applied.yaml</code>；这是 apply 从 configs/isaac.yaml 生成的版本配置。先在源配置明确 fix_base=true/false、gravity、physics_dt，机构与各连杆惯性须确认。闭环提供已验证的约束层和工程来源；默认空列表仍待确认。tool_frame只定义测量坐标，真实工具质量/碰撞要另在模型中建好。</p>
<pre>bash 启动工具包.sh run --project projects/旭创现场项目 inertia
bash 启动工具包.sh run --project projects/旭创现场项目 isaac
bash 启动工具包.sh run --project projects/旭创现场项目 asset</pre>
<p>runtime 是零重力、驱动禁用的专门软件检查，需要单独生成对应测试场景；填写 joint_positions 与 runtime_kinematics 容差，不能把真实动态场景直接当运行时FK测试场景。位姿/质量检查不证明动力学已对齐。</p>
<p><b>动力学：</b>按匹配固定基座、负载、控制模式、初始状态、实际输入和公共 test-relative 时间基准采集，仿真独立响应，再比较 q/dq/力矩/接触。不断把仿真状态设成实测状态只能检查回放，不能证明控制响应正确。</p>
<pre>bash 启动工具包.sh run --project projects/旭创现场项目 dynamics-fit
bash 启动工具包.sh run --project projects/旭创现场项目 drives-fit
bash 启动工具包.sh apply-drives --project projects/旭创现场项目
bash 启动工具包.sh run --project projects/旭创现场项目 contact-fit
bash 启动工具包.sh run --project projects/旭创现场项目 response</pre>
<p>dynamics-fit 只在已知刚体惯性基础上拟合附加惯量/粘性/库仑摩擦/偏置，不辨识所有连杆质量/惯性。apply-drives 需要先给出厂家允许运行 max_effort，只生成候选配置；改变 job 到候选配置后在新场景目录重导，再做留出响应试验。接触不同模型分开采样和拟合；静摩擦、延迟等未辨识项应保留待完成。</p>
<p>sysid job 默认生成 NVIDIA 官方调用计划；真正执行用底层 run-nvidia-sysid --execute，并按 configs/nvidia_sysid.yaml 和官方输入要求准备遥测/资产/激励。现场统一入口不开放 execute。官方运行日志成功也不自动意味着符合旭创精度。</p>
<p><b>MuJoCo：</b>可作为无需Isaac的模型打开/FK软件验证路线，不能替代D03 Isaac交付。本包含MuJoCo依赖；运行程序说明见 docs/mujoco_validation.md。示例要求明确指定模型、输出和关节参考；DAE会转换，原URDF保留。自带的仿真回放不发送硬件命令。</p>
<pre>bash 启动工具包.sh --help
# 一键检查项目模型；有桌面环境时加 --viewer 打开查看器：
bash 启动工具包.sh preview --project projects/旭创现场项目 --revision VIEW01
# 查看MuJoCo的全部参数（使用包内Python）：
runtime/python/bin/python3 examples/run_mujoco_validation.py --help
runtime/python/bin/python3 examples/view_mujoco.py --help</pre>''')
    section('delivery','10　怎样验收、如何交付、怎样恢复', '''
<p>报告应含仪器型号/校准/能力、装夹与坐标依据、采集范围和工况、原始与转换数据、标定和另采验证集、前后位置/方向最大值与RMS、限位重复性、动态/接触对比、模型/数据/配置版本和未完成项。小残差不能替代独立测量；所有 synthetic/simulation 记录都不能升级为真机验收。</p>
<pre>bash 启动工具包.sh run --project projects/旭创现场项目 report
bash 启动工具包.sh status --project projects/旭创现场项目
bash 启动工具包.sh export --project projects/旭创现场项目 --destination /tmp/旭创_R01_阶段交付.zip
bash 启动工具包.sh backup --project projects/旭创现场项目 --destination /tmp/旭创_R01_完整项目.zip</pre>
<p>export 生成 D01—D06 分类归档，默认阶段交付，D03未做时附readiness，不伪装成仿真完成。正式阶段由 configs/delivery.yaml 的 stage 决定；覆盖动力学时选 dynamics，并明确 required_dynamics_metrics。确认相应真实独立验收报告与全部交付范围后使用 export --final；程序的接受门槛仍只是必要检查，须人工逐项签署测量与交付记录。完整源码/运行环境以工具包为准，分类ZIP用于审阅；复运行用完整项目备份加同版本工具包，保持项目内部目录关系。</p>
<p><b>最终一起交：</b>①完整项目ZIP（原始资料/参数/数据/模型/配置/报告）；②同版本离线工具包与MANIFEST、运行依赖锁定和第三方许可；③D01—D06分类ZIP；④适用的厂家/仪器规程与校准证据；⑤现场安装条件及已做/未做的实机和Isaac验证记录。原始CAD、机器人数据通过约定渠道交接，不随公开GitHub源码上传。</p>
<p><b>恢复：</b>解压同版本工具包和完整项目；运行 doctor 与自检；项目放任何可写目录，命令 --project 指向它；只更新该机 .machine.json 中外部SDK位置。新报告输入路径相对报告目录并校验原始哈希，不需要搜索替换 /home/用户名。老版本报告可能仍有绝对路径；保留原证据，在新版本重新运行验证生成可搬迁报告，不手改报告与哈希。</p>
<p><b>改动后的复验顺序：</b>先保存旧参数/模型/报告 → 改主表及来源 → 新版本结构检查和名义模型 → 若坐标/结构影响旧标定，重测或重拟合并留出验证 → apply / verify-applied → 重导Isaac与静态检查 → 同工况动态/接触留出验证 → 报告与分类/完整备份。</p>''')
    section('errors','11　看到什么结果算成功，失败后怎么处理', table(['结果/错误','含义','处理'],[
['ready / built / computed','结构准备好/文件生成/数学计算完成','不是全部验收通过，继续独立验证'],
['passed','该命令、该数据性质、该阈值下通过','检查是否真实数据、覆盖和测量能力，不能扩展到未测范围'],
['synthetic_pass / simulation_pass','合成或仿真软件检查通过','只能证明对应软件链路'],
['unverified / pending / draft，退出码3','缺阈值/未确认/草稿','补依据与配置后用新版本运行'],
['blocked / failed，退出码2','条件不足或不达标','查看results中的JSON/error与同名.log；保留失败证据'],
['Missing or changed evidence','数据或参数被修改、漏拷贝或报告太旧','恢复原件或按新版本完整重跑，不能改SHA绕过'],
['rank-deficient / not identifiable','所选参数耦合或采样不够','收缩自由参数；保持基准坐标；按批准工况扩大姿态覆盖并另采验证'],
['out of range / no extrapolation','反馈定义、零位、范围或域覆盖不一致','核对厂家定义与独立测量；不自动翻符号/裁值/扩限位'],
['already mapped / offsets applied','同一修正重复使用','回到原始输入，确定比例/零偏/几何各应用一次'],
['缺Isaac/SDK/设备适配','此机尚未配置或该型号尚未接入','使用厂家CSV导出继续离线计算；按官方安装对应SDK后实测'],
['Permission denied / noexec','硬盘挂载禁止执行或文件无执行权限','复制到本地可写可执行目录；shell脚本可用bash调用；不要改系统权限规避园区策略']]))
    section('programs','12　统一任务与原程序对应（无遗漏）',table(['现场job','原CLI程序','配置位置'],[[name,v['command'],'configs/jobs.yaml → jobs.'+name] for name,v in jobs().items()])+'''
<p>其余原程序 preflight/check-openarm/prepare-inputs/capture-data/run-test/map-joint-feedback/package-delivery 等仍可通过 run.py 调用；平台/设备不适用时明确标为不适用。原23条命令及完整参数列在后面，不能以不适用为理由省略交付范围内的功能。</p>'''+''.join('<details open><summary>'+h(name)+' — '+h(desc)+'</summary><pre>'+h(parser()._subparsers._group_actions[0].choices[name].format_help())+'</pre></details>' for name,(_,desc) in COMMANDS.items()))
    appendix=[]
    for sheet in ('阅读说明','参数工具矩阵','工具准备清单','交付物映射'):
        rows=catalog['sheets'][sheet]
        appendix.append('<h3>'+h(sheet)+'（保留完整原清单）</h3>'+table(rows[0],rows[1:]))
    appendix.append('<h3>程序主表的完整列名</h3>'+table(['Sheet','全部列名'],[[k,', '.join(v)] for k,v in SHEETS.items()]))
    section('appendix','13　完整附表：15类参数、11类工具、6类交付', '<p>以下保留R2原清单用于逐项核对。其中“时间配对待实现”为旧版状态；R3已增加通用CSV双靶标坐标转换、时间配对、应用和搬迁验证，仍未替代未选定设备的SDK接入。</p>'+''.join(appendix))
    refs=[]
    for code,ref in catalog['references'].items():
        refs.append('<li id="'+h(code)+'"><b>'+h(code)+' '+h(ref['owner'])+'</b>：<a href="'+h(ref['url'],quote=True)+'">'+h(ref['title'])+'</a><br><small>'+h(ref['scope'])+'</small></li>')
    extras=[
        ('ROS2 JointState消息定义','https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg'),
        ('ros2_control SimpleTransmission换算','https://github.com/ros-controls/ros2_control/blob/jazzy/transmission_interface/include/transmission_interface/simple_transmission.hpp'),
        ('MuJoCo官方建模与URDF扩展','https://mujoco.readthedocs.io/en/3.17.0/modeling.html#urdf-extensions'),
        ('NVIDIA6.1系统要求','https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html'),
        ('NVIDIA6.1 Python安装与离线依赖准备依据','https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_python.html'),
        ('可搬迁Python运行环境来源','https://docs.astral.sh/python-build-standalone/'),
        ('pip download离线依赖下载','https://pip.pypa.io/en/stable/cli/pip_download/'),
    ]
    refs.extend('<li><a href="'+h(url)+'">'+h(title)+'</a></li>' for title,url in extras)
    section('links','14　所有资料链接与进一步阅读', '<p>以下为官方、厂家或原作者来源；离线时可阅读本手册中的已整理操作说明。网页内容可能更新，出发前下载所选设备允许离线使用的正式手册并记录版本。链接保留原地址。</p><ol>'+''.join(refs)+'</ol><p>工具包仓库：<a href="https://github.com/wangyuting824/robot-calibration-toolkit">wangyuting824/robot-calibration-toolkit</a>。公开仓库可供别人访问；本地硬盘和 file:// 链接不能供他人远程打开。文档应上传飞书附件或按组织权限分享，不把本地路径当共享链接。</p>')
    css='''body{font:16px/1.75 "Noto Sans CJK SC","Microsoft YaHei",sans-serif;color:#18334b;max-width:1180px;margin:30px auto;padding:0 24px}h1,h2,h3{color:#123f68}h2{border-bottom:2px solid #d9e6ef;padding-top:26px}nav{background:#edf4fa;padding:16px;display:flex;flex-wrap:wrap;gap:8px 20px}a{color:#125cc1;overflow-wrap:anywhere}pre,code{font-family:monospace;background:#eef3f7}pre{padding:16px;white-space:pre-wrap;overflow-wrap:anywhere}table{border-collapse:collapse;width:100%;font-size:13px}td,th{border:1px solid #c9d7e1;padding:8px;min-width:80px;vertical-align:top;overflow-wrap:anywhere}th{background:#e3eef7}.scroll{overflow:auto}details{border:1px solid #dce4eb;padding:8px;margin:8px 0}small{color:#4e6070}@page{size:A4 landscape;margin:15mm}@media print{body{max-width:none;font-size:10px;line-height:1.5;margin:0;padding:0}table{font-size:8px}td,th{min-width:0;padding:4px}.scroll{overflow:visible}h2{break-before:page}nav,button,details{display:none}a{color:inherit}pre{font-size:9px}}'''
    css=css.replace('nav,button,details{display:none}','nav,button{display:none}details{display:block}')+'@media print{#start h2{break-before:auto}}'
    toc='<nav>'+''.join('<a href="#'+name+'">'+h(title)+'</a>' for name,title,_ in sections)+'</nav>'
    content='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>旭创URDF标定现场操作手册</title><style>'+css+'</style><h1>旭创 URDF 标定与仿真对齐<br>现场操作手册</h1><p>R3 · 2026-11-03 · 不依赖 Codex · Ubuntu x86_64 离线工具包</p><button onclick="window.print()">打印/另存PDF</button>'+toc+''.join('<section id="'+n+'"><h2>'+h(t)+'</h2>'+b+'</section>' for n,t,b in sections)+'</html>'
    (ROOT/'docs/现场操作手册.html').write_text(content,encoding='utf-8')
    (ROOT/'离线使用先读.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>离线工具包</title><style>body{font:20px/1.8 sans-serif;max-width:900px;margin:50px auto}</style><h1>旭创 URDF 标定现场工具包</h1><p>完整解压到可写目录，不需要 Codex。先打开下面的手册，再运行自检。</p><p><a href="docs/现场操作手册.html">打开完整现场操作手册</a></p><p><a href="docs/现场参数与工具总表.xlsx">打开完整参数、工具和交付总表</a></p><pre>bash 启动工具包.sh doctor\nbash 启动工具包.sh status --project projects/旭创现场项目</pre><p>软件包包含离线Python和依赖；Isaac、GPU驱动、ROS与厂家SDK另装。本项目尚待旭创确认资料和实测，不代表已完成真机标定。</p></html>',encoding='utf-8')
    from openpyxl import Workbook
    from openpyxl.styles import Font,PatternFill,Alignment
    book=Workbook();book.remove(book.active)
    for name,rows in catalog['sheets'].items():
        sheet=book.create_sheet(name)
        for row in rows: sheet.append(row)
    s=book.create_sheet('现场任务与程序');s.append(['任务','原CLI','配置选项']);
    for name,v in jobs().items():s.append([name,v['command'],json.dumps(v['options'],ensure_ascii=False)])
    s=book.create_sheet('主表完整字段');s.append(['sheet','列名']);
    for name,fields in SHEETS.items():s.append([name,', '.join(fields)])
    s=book.create_sheet('补充官方链接');s.append(['来源','链接']);
    for code,ref in catalog['references'].items():s.append([code+' '+ref['title'],ref['url']])
    for title,url in extras:s.append([title,url])
    for sheet in book:
        sheet.freeze_panes='A2';sheet.auto_filter.ref=sheet.dimensions
        for c in sheet[1]:c.font=Font(bold=True,color='FFFFFF');c.fill=PatternFill('solid',fgColor='215A8E')
        for col in sheet.columns:
            sheet.column_dimensions[col[0].column_letter].width=30
            for cell in col:
                cell.alignment=Alignment(wrap_text=True,vertical='top')
                if isinstance(cell.value,str) and cell.value.startswith('https://'):
                    cell.hyperlink=cell.value;cell.style='Hyperlink'
    book.save(ROOT/'docs/现场参数与工具总表.xlsx')
    ids={row[0] for row in catalog['sheets']['参数工具矩阵'][1:]}
    assert ids=={f'M{i:02d}' for i in range(1,16)}
    assert len(COMMANDS)==23
    assert {row[0] for row in catalog['sheets']['工具准备清单'][1:]}=={f'T{i:02d}' for i in range(1,12)}
    print('Manual coverage: 15 parameter classes, 11 tool classes, D01–D06, 23 CLI commands, all original references retained.')


if __name__=='__main__':build()
