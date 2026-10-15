const int HANTEI=30;
const int MAX_STEPS=400;
const float PI=3.14159265359;
float power=-4.;
float scale=1.;

const float RAY_STEP = 0.01;
const float BOUND_RADIUS=2.;
const float ROOT_EPS = 0.001;
vec3 parrete(vec3 p){
  return 0.5+0.5*cos(u_time+p+vec3(0.0,2.0,4.0));
}
vec2 uniformer(vec2 val){
  return (2.*val-u_resolution)/u_resolution.y;
}
vec3 polar(vec2 z){
  return vec3(cos(z.x)*cos(z.y),sin(z.x)*cos(z.y),sin(z.y));
}
vec2 arg(vec3 p){
  float r=length(p);
  if(r==0.){
    return vec2(0);
  }
  return vec2(atan(p.y,p.x),asin(p.z/r));
}
vec3 mul(vec3 p,vec3 q){
  return length(p)*length(q)*polar(arg(p)+arg(q));
}
vec3 pow(vec3 p, float exponent){
  return pow(length(p), exponent)*polar(exponent*arg(p));
}
vec3 newton(vec3 p){
  return mul(p/power,pow(p,-power)+vec3(power-1.0,0.0,0.0));
}
vec4 mul(vec4 p,vec4 q){
  return vec4(p.w*q.xyz+p.xyz*q.w+cross(p.xyz, q.xyz),p.w*q.w-dot(p.xyz, q.xyz));
}
vec4 sphericalMover(vec2 p){
    float l=length(p);
    float s=l>1e-6?sin(l)/l:1.;
    return vec4(-p.y*s,p.x*s,0.,cos(l));
}
vec4 conj(vec4 p){
  return vec4(-p.xyz,p.w);
}
vec3 rotate(vec3 p, vec4 rotor) {
  return mul(mul(rotor,vec4(p,0.)),conj(rotor)).xyz;
}
float basinId(vec3 c, out float iteration) {
  vec3 p=c;
  iteration=0.0;
  for(int i=0; i<HANTEI; ++i){
    vec3 np=newton(p);//次のp
    float diff=length(np-p);
    if(diff<ROOT_EPS){
      break;
    }
    if(diff>100.0){
      //実はこのdiffの範囲をいじることで構造を分解できる
      //計算量を減らすために発散点の判定は必須
      return -1.;
    }
    p=np;
    iteration++;
  }
  //収束先の根は単位球面上
  //p^a-1=0
  //p^a=1であるため
  float r=length(p);
  if(abs(r-1.)>0.01){
    //この闘値もいじると面白いことになる。
    return -1.0;
  }
  vec2 a=arg(p);
  float az=mod(floor(a.x*power/(2.*PI)+0.5)+power,power);
  float el=floor(a.y*power/(2.*PI)+0.5);
  return 1.0+az+(el+power)*power;
}
//レイと球の交差区間を求める
bool raySphere(vec3 ro,vec3 rd,float radius,out float t0,out float t1){
  float b=dot(ro,rd);
  float c=dot(ro,ro)-radius*radius;
  float h=b*b-c;
  if(h<0.){
    t0=0.;
    t1=0.;
    return false;
  }
  float s=sqrt(h);
  t0=-b-s;
  t1=-b+s;
  return t1>=0.0;
}
// 盆地IDが変わる箇所を探索する
bool rayMarch(vec3 ro,vec3 rd,out vec3 hitPos,out float hitT,out float iteration,out float rootA,out float rootB){
  float t0;
  float t1;
  if(!raySphere(ro,rd,BOUND_RADIUS,t0,t1)){
    return false;
  }
  t0=max(t0,0.);
  float prevT=t0;
  float prevIter;
  float prevId=basinId(ro+rd*prevT,prevIter);
  for(int i=1; i<=MAX_STEPS; ++i){
    float t=t0+float(i)*RAY_STEP;
    if(t>t1){
      break;
    }
    float it;
    float id=basinId(ro+rd*t,it);
    if(id!=prevId){
      float lo=prevT;
      float hi=t;
      float leftId=prevId;
      //境界がある区間だけ二分探索する
      for (int j=0; j<5; ++j) {
        float mid=0.5*(lo+hi);
        float midIter;
        float midId=basinId(ro+rd*mid,midIter);
        if(midId==leftId){
          lo=mid;
        }else{
          hi=mid;
        }
      }
      hitT=0.5*(lo+hi);
      hitPos=ro+rd*hitT;
      iteration=0.5*(prevIter+it);
      rootA=prevId;
      rootB=id;
      return true;
    }
    prevT=t;
    prevId=id;
    prevIter=it;
  }
  hitPos=vec3(0.0);
  hitT=0.0;
  iteration=0.0;
  rootA=-1.0;
  rootB=-1.0;
  return false;
}
// 盆地IDから色を生成
vec3 rootColor(float id){
  if(id<0.0){
    return vec3(0.12,0.025,0.18);
  }
  float h = id * 0.61803398875;
  return 0.5+0.5*cos(2.0*PI*h+vec3(0.0,2.0,4.0));
}
void mainImage(out vec4 fragColor, in vec2 fragCoord){
    vec2 uv=uniformer(fragCoord)*exp(-scale);
    vec4 quat=sphericalMover(uniformer(u_mouse)+u_time/10.0);
    vec3 ro=rotate(vec3(0.0,0.0,-3.0),quat);
    vec3 rd=normalize(rotate(vec3(uv,1.0),quat));
    vec3 hitPos;
    float hitT;
    float iteration;
    float rootA;
    float rootB;
    vec3 col;
    if(rayMarch(ro,rd,hitPos,hitT,iteration,rootA,rootB)){
        vec3 base=(rootColor(rootA)+rootColor(rootB))*0.5;
        //計量ライティング
        float r=length(hitPos);
        vec3 pseudoNormal=r>1e-5?hitPos/r:vec3(0.0, 0.0, -1.0);
        vec3 lightDir=normalize(vec3(0.6,-0.8,0.7));
        float diffuse=0.4+0.6*max(dot(pseudoNormal,lightDir),0.0);
        float specular=pow(max(dot(reflect(-lightDir, pseudoNormal),-rd),0.0),24.0);
        col=base*diffuse+0.12*specular;
        float fog=exp(-0.035*hitT*hitT);
        col=mix(parrete(hitPos),3.*col,fog);
    }
    fragColor = vec4(col,1.0);
}
