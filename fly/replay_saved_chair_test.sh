python3 - <<'PY'
import base64,hashlib,io,json,statistics
from pathlib import Path
import numpy as np,requests
from PIL import Image
root=Path('/home/nvidia/agile/fly/runtime/vision_checks')
groups=[('image_left','1559dcc18c60440081c417f678dc90a1'),('image_right','4dea97ad904e46a5bf30395ccc3ea0de')]
instruction='fly to the chair'
proprio=[0.05268628046740683,-0.025557892137450897,-0.22839999999999996,0.3383751820507681]
s=requests.Session();s.trust_env=False
rows=[]
for label,directory in groups:
 summary=json.loads((root/directory/'summary.json').read_text())
 for index in [0,3,7]:
  source=summary['rows'][index]
  image=Image.open(source['camera_file']).convert('RGB').resize((224,224));buf=io.BytesIO();image.save(buf,format='JPEG',quality=85);jpeg=buf.getvalue()
  assert hashlib.sha256(jpeg).hexdigest()==source['model_jpeg_sha256']
  for repeat in range(2):
   r=s.post('http://127.0.0.1:15017/predict',json={'image':base64.b64encode(jpeg).decode(),'instr':instruction,'proprio':proprio,'request_id':950000+len(rows)},timeout=(3,15))
   if r.status_code!=200:
    row={'label':label,'source_request_id':source['request_id'],'repeat':repeat+1,'http_status':r.status_code,'error':r.text[:300]}
   else:
    data=r.json();a=np.asarray(data['action_ori'],dtype=float).reshape(-1).tolist()
    row={'label':label,'source_request_id':source['request_id'],'repeat':repeat+1,'image_sha256':hashlib.sha256(jpeg).hexdigest(),'action_ori_cm_rad':a,'raw_response':data}
   rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
stats={}
for label,_ in groups:
 a=[r['action_ori_cm_rad'] for r in rows if r['label']==label and 'action_ori_cm_rad' in r]
 stats[label]={'samples':len(a),'median':[statistics.median(x[i] for x in a) for i in range(4)] if a else None,'y_min_cm':min(x[1] for x in a) if a else None,'y_max_cm':max(x[1] for x in a) if a else None}
result={'instruction':instruction,'fixed_proprio_cm_deg':proprio,'stats':stats,'rows':rows,'scope':'read-only response to image changes; not object localization or flight validation'}
(root/'actual_left_right_fixed_input.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(stats,ensure_ascii=False))
PY
