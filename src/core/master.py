import yaml
import logging
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

logger = logging.getLogger(__name__)

class DefaultNormalizer:
    """
    用于将操作员输入（例如遥操作手柄的位移/角度）标准化为机器人期望的关节或TCP增量。
    具体算法待后续实现，目前提供接口框架。
    """
    def __init__(self, config_path=None):
        self.input_range = [-1.0, 1.0]   # 输入范围（示例）
        self.output_scale = 0.1          # 输出比例因子

        if config_path:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = {}

        logger.info("DefaultNormalizer initialized with config: %s", self.config)

    def normalize_position(self, raw_value):
        """
        将原始位置数据映射为机器人关节/位姿增量
        :param raw_value:  float 或 list，从操作员端接收的输入
        :return:          float 或 list，机器人可用的命令增量
        """
        # 占位实现：直接乘以比例因子
        if isinstance(raw_value, list):
            return [v * self.output_scale for v in raw_value]
        return raw_value * self.output_scale

    def normalize_velocity(self, raw_value):
        """
        将原始速度映射为机器人关节/位姿速度
        """
        if isinstance(raw_value, list):
            return [v * self.output_scale * 0.5 for v in raw_value]  # 速度缩放更保守
        return raw_value * self.output_scale * 0.5