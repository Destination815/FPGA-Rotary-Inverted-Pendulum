from search import *
choices=[[ki,.35,tau,kp,ff,0]for ki in [.1,.15,.2,.25]for tau in [.5,1.,2.]for kp in [.5,.75,1.]for ff in [-.5,0]]
with concurrent.futures.ThreadPoolExecutor(max_workers=3)as ex:res=list(ex.map(runparam,enumerate(choices)))
(P/'refine_results.json').write_text(json.dumps(res,indent=2));best=sorted(res,key=lambda x:(-x['complete'],-x['strict'],x['worst_position_deg']))[:10];(P/'refine_best.json').write_text(json.dumps(best,indent=2));print('BEST',[(x['index'],x['p'],x['complete'],x['strict'],x['worst_position_deg'])for x in best])
