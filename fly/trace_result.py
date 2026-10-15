"""Validate and present coordinates already computed by the live VLA bridge."""
import math


def finite_vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError('%s 长度应为 %d' % (name, size))
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError('%s 包含无效数值' % name)
        result.append(float(item))
    return result


def make_result(trace, validation=None, rejection=None):
    if trace.get('control_publishing_configured') is not False:
        raise ValueError('桥接配置了控制发布，拒绝作为本次只读验证')
    if trace.get('preview_pair_alignment') is not True:
        raise ValueError('本工具要求现有桥接使用配对位姿预览')
    xyz = finite_vector(trace.get('target_mavros_xyz_m'), 3, 'MAVROS 目标')
    delta = finite_vector(trace.get('target_minus_paired_mavros_m'), 3, '目标相对偏移')
    yaw = finite_vector([trace.get('target_mavros_yaw_deg')], 1, '目标 yaw')[0]
    dyaw = finite_vector([trace.get('target_minus_paired_mavros_yaw_deg')], 1, 'yaw 偏移')[0]
    finite_vector(trace.get('server_action_cm_rad'), 4, '服务器 action')
    finite_vector(trace.get('server_action_ori_cm_rad'), 4, '服务器 action_ori')
    error = finite_vector([trace.get('server_composition_max_error')], 1, '坐标组合误差')[0]
    if not 0 <= error <= 1e-3:
        raise ValueError('服务器坐标组合检查失败')
    stamp = finite_vector([trace.get('image_stamp')], 1, '图像时间')[0]
    if not isinstance(trace.get('request_id'), int) or not trace.get('run_id'):
        raise ValueError('缺少请求关联标识')
    if validation is not None:
        for field in ('run_id', 'request_id', 'instruction'):
            if validation.get(field) != trace.get(field):
                raise ValueError('预览验证与坐标不是同一次请求')
        if abs(float(validation['observation_stamp']) - stamp) > 1e-5:
            raise ValueError('预览验证与坐标图像时间不同')
        v_xyz = finite_vector(validation.get('target_xyz_m'), 3, '通过检查的目标')
        if max(abs(a - b) for a, b in zip(xyz, v_xyz)) > 1e-6:
            raise ValueError('通过检查的目标与原始坐标不同')
        if validation.get('target_frame') != trace.get('target_frame'):
            raise ValueError('验证坐标系不一致')
        v_yaw = finite_vector([validation.get('target_yaw_rad')], 1, '通过检查的 yaw')[0]
        if abs(math.atan2(math.sin(math.radians(yaw) - v_yaw),
                          math.cos(math.radians(yaw) - v_yaw))) > 1e-6:
            raise ValueError('验证 yaw 不一致')
    return {
        'instruction': trace['instruction'],
        'run_id': trace['run_id'], 'request_id': trace['request_id'],
        'target_frame': trace['target_frame'], 'position_unit': 'm', 'yaw_unit': 'deg',
        'target': dict(zip(('x', 'y', 'z', 'yaw'), xyz + [yaw])),
        'relative_to_observation': dict(zip(('dx', 'dy', 'dz', 'dyaw'), delta + [dyaw])),
        'bridge_check': 'passed' if validation else ('rejected' if rejection else 'unconfirmed'),
        'rejection': rejection, 'control_published_by_fly': False,
        'physical_axes_calibrated': False, 'physical_target_reached': False,
        'trace': trace, 'validated_prediction': validation,
    }
