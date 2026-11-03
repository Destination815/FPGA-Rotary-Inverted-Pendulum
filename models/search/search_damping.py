from search import *
set_backend('damping')
for s,_,_ in prepared:s['kick']=.3
choices=[[.25,.35,1.,.75,-.5,0,at,aw,fr]for at in [.7,.85,1.]for aw in [1.,1.15,1.3]for fr in [.5,1.]]
with concurrent.futures.ThreadPoolExecutor(max_workers=3)as ex:res=list(ex.map(runparam,enumerate(choices)))
(P/'damping_results.json').write_text(json.dumps(res,indent=2));best=sorted(res,key=lambda x:(-x['complete'],-x['strict'],x['worst_position_deg']))[:10];(P/'damping_best.json').write_text(json.dumps(best,indent=2));print('BEST',[(x['index'],x['p'],x['complete'],x['strict'],x['worst_position_deg'])for x in best])
