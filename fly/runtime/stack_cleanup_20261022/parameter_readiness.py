"""Wait for MAVROS' read-only parameter cache; never replace missing values."""
import time


def read_required_parameters(get,names,guard,timeout=15.,clock=None,sleep=None,on_wait=None):
    clock=time.monotonic if clock is None else clock
    sleep=time.sleep if sleep is None else sleep
    deadline=clock()+timeout;announced=False
    while True:
        values={};missing=[]
        for name in names:
            guard()
            try:
                reply=get(param_id=name)
                if reply.success:
                    values[name]={'integer':reply.value.integer,'real':reply.value.real}
                else:missing.append(name)
            except Exception as exc:
                missing.append(name+' ('+type(exc).__name__+')')
        guard()
        if not missing:return values
        remaining=deadline-clock()
        if remaining<=0:
            raise RuntimeError('等待飞控参数就绪超时，仍无法读取：'+', '.join(missing)+'；未使用默认值或修改飞控参数')
        if not announced and on_wait:
            on_wait('飞控参数尚未就绪，等待重试（最多%.0f秒）：%s'%(timeout,', '.join(missing)))
            announced=True
        sleep(min(.5,remaining))
