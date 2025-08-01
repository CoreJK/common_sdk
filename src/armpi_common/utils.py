def split_to_bytes(value):
    """大端转换成小端，将数值拆分为低8位和高8位，低8位在前"""
    low_byte = value & 0xFF
    high_byte = (value >> 8) & 0xFF
    return [low_byte, high_byte]

def calculate_checksum(data: list):
    """计算帧头之后数据的校验和, 然后按位取反, 取最后8位"""
    sum_from_2 = sum(data[2:])
    checksum = ~sum_from_2 & 0xFF
    return checksum

def get_command_info_by_id(cmd_id, cmd_table: dict):
    """根据命令ID获取命令信息"""
    for cmd_name, cmd_data in cmd_table.items():
        if cmd_data[4] == cmd_id:  # 第5个元素是命令ID
            return cmd_name  # 返回命令名