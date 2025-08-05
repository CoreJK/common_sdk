# -*- coding: utf-8 -*-
import copy
import logging
import logging.handlers

__all__ = ['logger', 'set_stream_level', 'set_file_level', 'disable_logging']


# 日志等级映射
LEVELS = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARNING': logging.WARNING,
    'ERROR': logging.ERROR,
    'CRITICAL': logging.CRITICAL
}


class ColorfulFormatter(logging.Formatter):
    """终端显示日志的格式化器"""
    COLORS = {
        'WARNING': '33',         # 黄色
        'INFO': '32',            # 绿色
        'DEBUG': '34',           # 蓝色
        'CRITICAL': '35',        # 紫色
        'ERROR': '31',           # 红色
        'NAME_AND_LINENO': '36'  # 青色
    }

    def format(self, record):
        colored_record = copy.copy(record)
        levelname = colored_record.levelname
        seq = self.COLORS.get(levelname, '37')  # 默认为白色
        colored_levelname = f"\033[1;{seq}m{levelname.ljust(9)}\033[0m"  # 1 表示样式为粗体，ljust(9) 使得所有的 levelname 都有相同的长度
        colored_record.levelname = colored_levelname
        colored_record.msg = f"\033[5;{seq}m{colored_record.msg}\033[0m"  # 设置消息的颜色
        colored_record.name = f"\033[1;{self.COLORS['NAME_AND_LINENO']}m{colored_record.name}:{colored_record.funcName}:{colored_record.lineno}\033[0m"  # 设置 logger 名称和行号的颜色
        
        return super().format(colored_record)


class LoggerFileFormatter(logging.Formatter):
    """日志文件的格式化器"""
    def format(self, record):
        colored_record = copy.copy(record)
        levelname = colored_record.levelname
        colored_levelname = f"{levelname.ljust(7)}"  # 1 表示样式为粗体，ljust(9) 使得所有的 levelname 都有相同的长度
        colored_record.name = f"{colored_record.name}:{colored_record.funcName}:{colored_record.lineno}"
        colored_record.levelname = colored_levelname
        
        return super().format(colored_record)

FORMAT = '%(asctime)s | %(levelname)s | %(name)s - %(message)s'
logger = logging.getLogger("kwrt_robot_arm")

# 输出到终端的日志处理器
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(ColorfulFormatter(FORMAT))  # 日志格式化器
logger.addHandler(stream_handler)

# 输出到文件的日志处理器
file_handler = logging.handlers.RotatingFileHandler('kwrt_robot_arm.log', maxBytes=1024*1024, backupCount=5, encoding='utf-8')
file_handler.setFormatter(LoggerFileFormatter(FORMAT))  # 日志格式化器
logger.addHandler(file_handler)
logger.setLevel(logging.DEBUG)


def set_stream_level(level_str='INFO'):
    """设置终端日志处理器的日志等级"""
    level = LEVELS.get(level_str.upper(), logging.INFO)
    stream_handler.setLevel(level)

def set_file_level(level_str='INFO'):
    """设置文件日志处理器的日志等级"""
    level = LEVELS.get(level_str.upper(), logging.DEBUG)
    file_handler.setLevel(level)

def disable_logging(level_str='CRITICAL'):
    """禁用指定等级及以下的日志输出"""
    level = LEVELS.get(level_str.upper(), logging.CRITICAL)
    logging.disable(level)