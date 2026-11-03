#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <limits>
struct DampingGate {
 int quiet=0,sign=0,elapsed=0,halves=0,weight=0,ramp=0;bool active=false,mode=false;
 void clear(){quiet=sign=elapsed=halves=weight=ramp=0;active=false;mode=false;}
 int tick(bool eligible,int theta,bool postmove=false){
  if(!eligible||mode!=postmove){clear();mode=postmove;return weight;}if(quiet<1000){quiet++;return weight;}bool was=active;
  const int lower=postmove?60:80,upper=postmove?500:180;
  if(!active){if(sign)elapsed=std::min(upper+1,elapsed+1);int hit=theta>=1144?1:theta<=-1144?-1:0;
   if(hit&&hit!=sign){halves=sign&&elapsed>=lower&&elapsed<=upper?std::min(4,halves+1):0;sign=hit;elapsed=0;if(halves>=4)active=true;}
   else if(elapsed>upper){sign=elapsed=halves=0;}}
  if(was&&weight<3){ramp++;if(ramp>=20){ramp=0;weight++;}}return weight;
 }
};

struct HoldObserver {
int r43_variant=0;
bool r40_unwind_active=false;
// stasis_reference mirrors the pre-existing RTL reference_s latch. Only
// stasis_error32 and stasis_samples8 are additional hardware registers.
int32_t stasis_error=0,stasis_reference=0;unsigned stasis_samples=0;
bool rate_good=false;unsigned fast_good=0;int64_t fast_correction=0,rate_replacement_delta=0;
DampingGate damping_gate;
static constexpr int64_t Q=16777216;
int64_t matrix[7][11]={{11286646, -31967, -16442, 102789, -184055, 272, 1386, -546481, -28, 5641830, 16442}, {-34177534, 15381777, -87634, 643490, -1154404, -40426, -252652, -3418282, -42, 35277151, 87634}, {-83134, -2871, 15690392, 158363, -2279, 6, 42, -8022, -21, 85266, 1086824}, {-747153, 3020, -904581, 16760913, 62472, 29, 119, -3103, -4383, 736356, 904581}, {-25403164, -920498, 5012, 476306, 2857938, 1961, 13023, 8460885, -130, 26099228, -5012}, {-2054030, 786677, 99401, -569297, 2336525, 11974284, -6886, 1919669, -125392, 1215306, -99401}, {-1384029, 87381, 97028, -69994, 435582, 4037998, 11974831, 117084, -19965, 1279261, -97028}};
int64_t brakegain[7]={67937104, 38753879, 3568142, 30194271, -268429, -2104864, -2148228};
int64_t gain[7]={126466217, 117243313, 11515862, 91736295, -747125, -5912648, -7009661};
int64_t again[7]={67937104, 38753879, 3568142, 30194271, -268429, -2104864, -2148228};
int64_t dead=298096;
int64_t ratio=20906080;
int64_t inverse_ratio=13463786;
int64_t friction=475651;
int64_t theta_fast_gain=180666024;
int64_t table[1025]={0,163835,327638,491379,655027,818550,981917,1145097,1308060,1470775,1633211,1795340,1957129,2118551,2279574,2440171,2600313,2759970,2919116,3077721,3235759,3393203,3550027,3706203,3861708,4016514,4170599,4323937,4476506,4628281,4779241,4929363,5078627,5227010,5374493,5521057,5666681,5811348,5955040,6097739,6239428,6380092,6519716,6658283,6795781,6932195,7067513,7201722,7334811,7466768,7597583,7727246,7855748,7983080,8109234,8234203,8357980,8480558,8601932,8722097,8841048,8958781,9075292,9190579,9304639,9417471,9529072,9639442,9748580,9856487,9963162,10068608,10172825,10275814,10377580,10478123,10577447,10675556,10772453,10868142,10962629,11055917,11148012,11238920,11328646,11417197,11504578,11590797,11675860,11759776,11842550,11924190,12004706,12084103,12162392,12239580,12315676,12390689,12464628,12537501,12609319,12680090,12749824,12818531,12886220,12952901,13018584,13083280,13146997,13209747,13271539,13332383,13392291,13451272,13509337,13566495,13622758,13678135,13732638,13786277,13839061,13891002,13942110,13992394,14041867,14090537,14138416,14185512,14231838,14277402,14322215,14366288,14409629,14452249,14494158,14535366,14575883,14615718,14654881,14693381,14731228,14768432,14805002,14840947,14876276,14910999,14945124,14978660,15011617,15044003,15075826,15107096,15137821,15168009,15197669,15226808,15255436,15283560,15311187,15338327,15364986,15391173,15416895,15442159,15466973,15491344,15515280,15538787,15561873,15584544,15606808,15628671,15650139,15671220,15691920,15712246,15732202,15751797,15771036,15789924,15808469,15826676,15844550,15862097,15879323,15896234,15912835,15929131,15945128,15960830,15976244,15991373,16006223,16020800,16035106,16049149,16062931,16076458,16089734,16102764,16115552,16128102,16140419,16152506,16164369,16176010,16187434,16198644,16209646,16220441,16231035,16241430,16251631,16261641,16271463,16281100,16290557,16299836,16308940,16317873,16326639,16335239,16343677,16351956,16360079,16368049,16375868,16383540,16391067,16398451,16405696,16412804,16419777,16426618,16433330,16439914,16446374,16452711,16458928,16465027,16471010,16476880,16482638,16488287,16493828,16499264,16504596,16509827,16514958,16519992,16524930,16529773,16534525,16539185,16543757,16548242,16552641,16556956,16561189,16565341,16569414,16573409,16577327,16581171,16584941,16588639,16592266,16595824,16599314,16602737,16606094,16609388,16612618,16615786,16618894,16621942,16624932,16627864,16630740,16633561,16636328,16639042,16641703,16644314,16646875,16649386,16651849,16654265,16656635,16658959,16661238,16663474,16665666,16667817,16669926,16671995,16674024,16676014,16677965,16679879,16681757,16683598,16685404,16687175,16688912,16690615,16692286,16693925,16695532,16697108,16698654,16700170,16701657,16703115,16704545,16705948,16707323,16708672,16709995,16711293,16712566,16713814,16715038,16716238,16717415,16718570,16719702,16720813,16721902,16722970,16724018,16725045,16726053,16727041,16728010,16728960,16729892,16730806,16731703,16732582,16733444,16734290,16735119,16735932,16736730,16737512,16738279,16739031,16739769,16740492,16741202,16741898,16742580,16743249,16743906,16744549,16745181,16745800,16746407,16747002,16747586,16748158,16748720,16749271,16749811,16750340,16750860,16751369,16751869,16752359,16752839,16753310,16753772,16754225,16754670,16755106,16755533,16755952,16756363,16756766,16757162,16757549,16757929,16758302,16758668,16759026,16759378,16759723,16760061,16760393,16760718,16761037,16761350,16761656,16761957,16762252,16762541,16762825,16763103,16763376,16763644,16763906,16764164,16764416,16764663,16764906,16765144,16765378,16765606,16765831,16766051,16766267,16766479,16766686,16766890,16767090,16767285,16767477,16767666,16767850,16768031,16768209,16768383,16768554,16768721,16768886,16769047,16769205,16769360,16769512,16769661,16769807,16769950,16770090,16770228,16770363,16770496,16770626,16770753,16770878,16771001,16771121,16771239,16771354,16771468,16771579,16771688,16771795,16771900,16772003,16772103,16772202,16772299,16772394,16772488,16772579,16772669,16772757,16772843,16772927,16773010,16773092,16773171,16773250,16773326,16773402,16773475,16773548,16773619,16773688,16773756,16773823,16773889,16773953,16774016,16774078,16774139,16774199,16774257,16774314,16774370,16774425,16774479,16774532,16774584,16774635,16774685,16774734,16774782,16774829,16774875,16774920,16774965,16775008,16775051,16775093,16775134,16775174,16775214,16775252,16775290,16775328,16775364,16775400,16775435,16775470,16775503,16775536,16775569,16775601,16775632,16775663,16775693,16775722,16775751,16775779,16775807,16775834,16775861,16775887,16775913,16775938,16775963,16775987,16776011,16776034,16776057,16776080,16776102,16776123,16776144,16776165,16776185,16776205,16776225,16776244,16776263,16776281,16776299,16776317,16776334,16776351,16776368,16776385,16776401,16776416,16776432,16776447,16776462,16776476,16776491,16776505,16776519,16776532,16776545,16776558,16776571,16776583,16776596,16776608,16776619,16776631,16776642,16776653,16776664,16776675,16776685,16776696,16776706,16776716,16776725,16776735,16776744,16776753,16776762,16776771,16776780,16776788,16776796,16776804,16776812,16776820,16776828,16776835,16776843,16776850,16776857,16776864,16776871,16776877,16776884,16776890,16776897,16776903,16776909,16776915,16776921,16776926,16776932,16776937,16776943,16776948,16776953,16776958,16776963,16776968,16776973,16776978,16776982,16776987,16776991,16776996,16777000,16777004,16777008,16777012,16777016,16777020,16777024,16777028,16777031,16777035,16777038,16777042,16777045,16777048,16777052,16777055,16777058,16777061,16777064,16777067,16777070,16777073,16777075,16777078,16777081,16777083,16777086,16777088,16777091,16777093,16777096,16777098,16777100,16777103,16777105,16777107,16777109,16777111,16777113,16777115,16777117,16777119,16777121,16777123,16777125,16777126,16777128,16777130,16777131,16777133,16777135,16777136,16777138,16777139,16777141,16777142,16777144,16777145,16777146,16777148,16777149,16777150,16777152,16777153,16777154,16777155,16777156,16777158,16777159,16777160,16777161,16777162,16777163,16777164,16777165,16777166,16777167,16777168,16777169,16777170,16777171,16777172,16777172,16777173,16777174,16777175,16777176,16777176,16777177,16777178,16777179,16777179,16777180,16777181,16777182,16777182,16777183,16777184,16777184,16777185,16777185,16777186,16777187,16777187,16777188,16777188,16777189,16777189,16777190,16777190,16777191,16777191,16777192,16777192,16777193,16777193,16777194,16777194,16777194,16777195,16777195,16777196,16777196,16777196,16777197,16777197,16777198,16777198,16777198,16777199,16777199,16777199,16777200,16777200,16777200,16777201,16777201,16777201,16777201,16777202,16777202,16777202,16777203,16777203,16777203,16777203,16777204,16777204,16777204,16777204,16777204,16777205,16777205,16777205,16777205,16777206,16777206,16777206,16777206,16777206,16777207,16777207,16777207,16777207,16777207,16777207,16777208,16777208,16777208,16777208,16777208,16777208,16777208,16777209,16777209,16777209,16777209,16777209,16777209,16777209,16777210,16777210,16777210,16777210,16777210,16777210,16777210,16777210,16777211,16777211,16777211,16777211,16777211,16777211,16777211,16777211,16777211,16777211,16777211,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777212,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777213,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777214,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777215,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216,16777216};
static constexpr int64_t motion_gains[8][8]={{67937104,38753879,3568142,30194271,-268429,-2104864,-2148228,-3019427},{76298406,49966655,4703531,38985989,-336814,-2648833,-2842718,-3786453},{84659708,61179432,5838919,47777706,-405199,-3192802,-3537209,-4553479},{93021010,72392208,6974308,56569424,-473584,-3736771,-4231699,-5320505},{101382311,83604984,8109696,65361142,-541970,-4280741,-4926190,-6087530},{109743613,94817760,9245085,74152860,-610355,-4824710,-5620680,-6854556},{118104915,106030537,10380473,82944577,-678740,-5368679,-6315171,-7621582},{126466217,117243313,11515862,91736295,-747125,-5912648,-7009661,-8388608}};
// R28 retains R27 zero initialization and natural 200ms warm / 200ms mix.
// R28 transaction state. tick() exposes the raw delivered PWM; policy_tick()
// additionally applies the exact automatic mix and is the RTL oracle entry.
bool manual_moved=false; bool drift_latched=false,motion_watch=false,motion_latched=false;
int64_t z[7]={},integral=0,lp=0,blend=0;
int age=0,samples=0,ready=0,fault=0,motion_bad_ticks=0,motion_phase=0,motion_ticks=0;
void clear(){r40_unwind_active=false;rate_replacement_delta=0;rate_good=false;fast_good=0;fast_correction=0;stasis_error=stasis_reference=0;stasis_samples=0;damping_gate.clear();manual_moved=false;drift_latched=motion_watch=motion_latched=false;for(auto&v:z)v=0;
 age=samples=ready=fault=motion_bad_ticks=motion_phase=motion_ticks=0;integral=lp=blend=0;}
static int64_t sat(int64_t a){return std::clamp<int64_t>(a,-2147483648LL,2147483647LL);}
static int64_t i32(int64_t a){uint32_t u=uint32_t(a);return u<0x80000000u?int64_t(u):int64_t(u)-4294967296LL;}
static int rounded_q24(int64_t value){int64_t mag=std::abs(value),n=mag>>24;
 n+=(mag%Q>Q/2)||(mag%Q==Q/2&&(n&1));return int(value<0?-n:n);}
int64_t lut(int64_t x){int64_t i=(std::abs(x)*10240+Q/2)>>24;return (x<0?-1:1)*(i<1025?table[i]:Q);}
// useA/ff retain the explicitly experimental tracking_dev interface. Production
// policy_tick calls useA==automatic; automatic feedforward always follows the
// frozen motion gain table, never the legacy caller's ff argument.
int tick(int t,int p,int previous,int cap,bool useA,int reference=0,int velocity=0,
 double ff=-.5,bool automatic=false,bool brake=false,
 int prediction_previous=std::numeric_limits<int>::max(),bool motion_assist_permission=false,bool mature_static_hold=false,
 int measured_omega=0,bool measurement_current=false,bool rate_hold=false){
 if(!automatic&&std::abs(int64_t(velocity))>64)manual_moved=true;
 if(stasis_samples&&stasis_reference!=reference)stasis_samples=0;stasis_reference=reference;
 if(!automatic||brake||!motion_assist_permission||!measurement_current||fault)r40_unwind_active=false;
 if(fault)return 0;
 if(std::abs(int64_t(t))>20000||std::abs(int64_t(p))>823550||std::abs(int64_t(previous))>350||cap<150||cap>350||std::abs(int64_t(reference))>823550){fault=1;r40_unwind_active=false;return 0;}
 const bool postmove_mode=mature_static_hold&&automatic&&!brake&&velocity==0&&motion_assist_permission&&motion_latched&&motion_phase==3;
 const bool damping_eligible=postmove_mode?(measurement_current&&std::llabs(int64_t(measured_omega))<163840&&std::abs(int64_t(t))<=9151):(mature_static_hold&&automatic&&!brake&&velocity==0&&!motion_latched&&std::abs(int64_t(t))<=9151&&std::abs(int64_t(p)-reference)<=5719);
 damping_gate.tick(damping_eligible,t,postmove_mode);
 if(automatic&&!brake&&std::abs(int64_t(velocity))>64)motion_watch=true;
 if(automatic&&!brake&&motion_assist_permission&&std::abs(int64_t(t))>1144){
  motion_bad_ticks=std::min(20,motion_bad_ticks+1);if(motion_bad_ticks>=20)motion_latched=true;
 }else motion_bad_ticks=0;
 const int prediction=(automatic&&prediction_previous!=std::numeric_limits<int>::max())?prediction_previous:previous;
 int64_t prior=prediction*16777LL,e=prior>=0?std::max<int64_t>(0,prior-dead):std::min<int64_t>(0,prior+dead);e=sat(e*(e<0?ratio:Q)>>24);
 int64_t in[11],n[7];for(int i=0;i<7;i++)in[i]=z[i];in[7]=e;in[8]=lut(z[3]);in[9]=int64_t(t)*256;in[10]=int64_t(p)*256;
 for(int i=0;i<7;i++){int64_t v=0;for(int j=0;j<11;j++)v+=(in[j]*matrix[i][j])>>24;n[i]=sat(v);}
 drift_latched=drift_latched||(automatic&&std::abs(int64_t(p)-reference)>5719);bool drift=automatic&&drift_latched;
 int64_t oldlp=lp;lp=sat(lp+(((int64_t(p)-reference)*256-lp)*(drift?133682:16760)>>24));
 if(!drift&&(std::abs(n[1])>838861||std::abs(n[3])>838861))lp=(!automatic&&manual_moved&&velocity==0&&std::abs(int64_t(t)*256)<2348810)?lp:(((automatic||std::abs(int64_t(reference))>2288)&&std::abs(int64_t(velocity))<6554)?oldlp:0);
 const bool stasis_eligible=mature_static_hold&&automatic&&!brake&&velocity==0&&!motion_latched&&std::abs(int64_t(t))<1144&&std::abs(int64_t(p)-reference)<=5719&&std::abs(n[1])<838861&&std::abs(n[3])<838861;
 const bool endpoint_stasis_eligible=mature_static_hold&&automatic&&!brake&&motion_assist_permission&&velocity==0&&measurement_current&&std::abs(int64_t(p)-reference)>1144&&std::abs(int64_t(t))<3432&&std::abs(n[1])<838861&&std::abs(n[3])<838861;
 const int32_t observed_error=int32_t(int64_t(p)-reference);
 if(stasis_eligible||endpoint_stasis_eligible)stasis_samples=(stasis_samples&&observed_error==stasis_error)?std::min(201U,stasis_samples+1):1;else stasis_samples=0;
 stasis_error=observed_error;
 const bool rate_eligible=(mature_static_hold||rate_hold)&&velocity==0&&automatic&&!brake&&measurement_current&&std::llabs(int64_t(measured_omega))<=1966080;
 const bool previous_rate_good=rate_good;rate_good=rate_eligible;
 const bool fast_eligible=mature_static_hold&&automatic&&!brake&&velocity==0&&!motion_latched&&!motion_assist_permission&&measurement_current&&std::llabs(int64_t(measured_omega))<=1966080;
 fast_good=fast_eligible?std::min(201U,fast_good+1):0;
 const int64_t innovation=int64_t(measured_omega)*256-10*n[1];
 const int64_t shaped=innovation>13421773?innovation-13421773:innovation< -13421773?innovation+13421773:0;
 const bool r38_boost=std::llabs(innovation)>13421773&&std::llabs(int64_t(measured_omega))<163840&&std::llabs(int64_t(t))<9830;
 const int64_t strong_shaped=(r43_variant==12||r43_variant==15||r43_variant==16)?shaped:(innovation>3355443?innovation-3355443:innovation< -3355443?innovation+3355443:0);
 fast_correction=rate_eligible&&previous_rate_good?(r38_boost?std::clamp<int64_t>(motion_assist_permission?(strong_shaped>>2):(shaped*4),-2097152,2097152):std::clamp<int64_t>(shaped>>4,-1048576,1048576)):0;
 const bool static_i_hold=stasis_eligible&&!endpoint_stasis_eligible&&stasis_samples>=201;
 const int64_t feedback_v=static_i_hold&&fast_good>=201?std::clamp<int64_t>(n[3],-50332,50332):n[3];
 const bool explicit_endpoint=endpoint_stasis_eligible /* R41: explicit endpoint correction tolerates encoder dithering */;
 rate_replacement_delta=postmove_mode&&measurement_current&&motion_latched&&motion_phase==3?std::clamp<int64_t>((innovation>>6)*damping_gate.weight,-243012,243012):0;
 // R41: release only the quiet-velocity gate while correcting an
 // actual lead of an explicitly moving reference. Unwind stays prior.
 const bool r41_moving_lead_i=automatic&&!brake&&motion_assist_permission&&measurement_current&&((velocity>64&&p>reference&&lp>0)||(velocity< -64&&p<reference&&lp<0));
 int64_t istep=lp*((r41_moving_lead_i||(drift&&!explicit_endpoint))?1048:4194)>>24,request=fast_correction;
 const bool r40_scope=automatic&&!brake&&motion_assist_permission&&measurement_current;
 const bool r40_opposed=lp*integral<0;
 const bool r40_trigger=r40_scope&&r40_opposed&&std::llabs(lp)>1218492&&std::llabs(integral)>=838861;
 r40_unwind_active=r40_scope&&integral!=0&&(r40_unwind_active||r40_trigger);
 const bool r40_unwind=r40_unwind_active&&r40_opposed;
 if(r40_unwind)istep=std::clamp<int64_t>(lp*8389>>24,-16777,16777);
 const bool r43_calm=automatic&&!brake&&velocity==0&&motion_assist_permission&&measurement_current&&std::abs(t)<6863;
 if(r43_calm&&(r43_variant==9||r43_variant==10||r43_variant==11)){request=0;rate_replacement_delta=0;}
 if(r43_calm&&r43_variant==13)request=0;
 if(r43_calm&&(r43_variant==14||r43_variant==15))rate_replacement_delta=0;
 for(int i=0;i<7;i++){int64_t g=useA?(brake?brakegain[i]:motion_gains[motion_phase][i]):gain[i];
  if(r43_calm&&useA){
   if(i==0&&(r43_variant==1||r43_variant==3||r43_variant==5||r43_variant==10))g=motion_gains[0][0];
   if(i==0&&(r43_variant==2||r43_variant==4||r43_variant==6||r43_variant==11))g=motion_gains[1][0];
   if(i==1&&r43_variant==3)g=motion_gains[0][1];
   if(i==1&&r43_variant==4)g=motion_gains[1][1];
   if(i==1&&(r43_variant==5||r43_variant==6||r43_variant==7))g=g*9/8;
   if(i==1&&r43_variant==8)g=g*5/4;
  }
  if(!useA&&i==0&&(std::abs(n[1])>2516582||std::abs(n[0])>1464088))g=theta_fast_gain;
  if(i==1 && mature_static_hold && automatic && !brake && velocity==0 && cap==150 && !motion_latched)g=(g*(20+damping_gate.weight))/20;
  const int64_t feedback=i==3?feedback_v:i==1?sat(n[1]+rate_replacement_delta):n[i];
  request+=((feedback-(i==2?int64_t(reference)*256:0))*g)>>24;}
 const int64_t ff_term=automatic?((int64_t(velocity)*256*((motion_latched&&!brake)?motion_gains[motion_phase][7]:-Q/2))>>24):llround(ff*velocity*256);
 request=sat(request+(lut(feedback_v)*friction>>24)+integral+ff_term);
 int64_t req=sat((request*(request<0?inverse_ratio:Q)>>24)+(request<0?-dead:request>0?dead:0));
 int wanted=rounded_q24(req*1000);
 const bool slew_quiet=velocity==0?std::abs(n[1])<838861:(measurement_current&&std::llabs(int64_t(measured_omega))<32768);
 int slew=(r43_variant!=16&&automatic&&motion_latched&&!brake&&std::abs(int64_t(t))<2621&&slew_quiet)?5:20;
 // Hard limit has priority when the budget falls faster than permitted slew.
 int u=std::clamp(std::clamp(std::clamp(wanted,-cap,cap),previous-slew,previous+slew),-cap,cap);
 int64_t err=req-u*16777LL;
 if(age>=1000&&std::abs(int64_t(t)*256)<2348810&&(r41_moving_lead_i||r40_unwind||((!automatic&&manual_moved&&velocity==0&&std::abs(int64_t(t)*256)<2348810)&&istep*integral<0)||(std::abs(n[1])<838861&&std::abs(n[3])<838861))&&(std::abs(err)<50332||istep*err<0)){int64_t next_i=std::clamp<int64_t>(integral+istep,-5872026LL,5872026LL);if(r40_unwind||((!automatic&&manual_moved&&velocity==0&&std::abs(int64_t(t)*256)<2348810)&&(std::abs(n[1])>=838861||std::abs(n[3])>=838861)))next_i=integral>=0?std::max<int64_t>(0,next_i):std::min<int64_t>(0,next_i);if(static_i_hold)next_i=integral>=0?std::clamp<int64_t>(next_i,0,integral):std::clamp<int64_t>(next_i,integral,0);integral=next_i;}
 if(integral==0)r40_unwind_active=false;
 if(automatic&&motion_latched&&!brake&&age>=199&&motion_phase<3){if(motion_ticks>=49){motion_ticks=0;motion_phase++;}else motion_ticks++;}
 age=std::min(1000,age+1);ready=samples>=200;samples=std::min(200,samples+1);for(int i=0;i<7;i++)z[i]=n[i];return u;
}
int mix(int delivered,int fallback,bool automatic,bool hold,int64_t&weight){
 if(fault)return 0;if(!automatic)return delivered;
 weight=(age>=200&&(hold||motion_latched))?std::min<int64_t>(Q,weight+83886):std::max<int64_t>(0,weight-1048576);
 return fallback+rounded_q24(int64_t(delivered-fallback)*weight);
}
int policy_tick(int t,int p,int previous,int cap,int reference=0,int velocity=0,
 bool automatic=false,bool hold=true,int fallback=0,bool brake=false,
 int prediction_previous=std::numeric_limits<int>::max(),bool motion_assist_permission=false,bool mature_static_hold=false,
 int measured_omega=0,bool measurement_current=false,bool rate_hold=false){
 int u=tick(t,p,previous,cap,automatic,reference,velocity,-.5,automatic,brake,prediction_previous,motion_assist_permission,mature_static_hold,measured_omega,measurement_current,rate_hold);
 return mix(u,fallback,automatic,hold,blend);
}
};
