from search import *
checks=[]
for campaign,backend in [('search','initial'),('refine','initial'),('refine_sensor','sensor'),('damping','damping')]:
 for scene,_,_ in prepared:scene['kick']=.3 if campaign=='damping' else 0
 set_backend(backend);old=json.loads((P/(campaign+'_results.json')).read_text())[0];new=runparam((old['index'],old['p']));diff=float(np.max(np.abs(np.array([r['metrics']for r in old['rows']])-np.array([r['metrics']for r in new['rows']]))));assert diff<1e-12 and new['strict']==old['strict'];checks.append(dict(campaign=campaign,max_metric_difference=diff))
print('PASS',checks)
