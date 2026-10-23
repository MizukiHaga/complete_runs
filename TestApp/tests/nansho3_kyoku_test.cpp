#include "Path.h"
#include "Settings.h"
#include "SpeedControl.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
static kyoku::Path path;
static void simulate(kyoku::Config c,kyoku::Pose start,const kyoku::Gate *gates,kyoku::Pose finish,bool mirror) {
  assert(path.plan(c,start,gates,finish));
  assert(path.size()>10);
  for(int i=0;i<path.size();++i) {
    assert(path.safe({path[i].p,path[i].yaw}));
    assert(std::isfinite(path[i].speed));
    if(i>0&&i<path.size()-1) assert(path[i].speed>0);
  }
  kyoku::Pose pose=start;int progress=0,crossed=0;bool done=false;
  float speed=0,lastLeft=0,lastRight=0;
  for(int step=0;step<30000;++step) {
    auto cmd=path.follow(pose,speed,progress);
    if(!cmd.valid) {std::fprintf(stderr,"invalid step=%d p=%d xy=%f,%f\n",step,progress,pose.p.x,pose.p.y);std::abort();}
    if(cmd.done) {done=true;break;}
    assert(cmd.left>=0&&cmd.right>=0);
    assert(cmd.left<=c.wheelSpeed+0.01f&&cmd.right<=c.wheelSpeed+0.01f);
    float change=c.acceleration*0.01f;
    float left=lastLeft+fmaxf(-change,fminf(change,cmd.left-lastLeft));
    float right=lastRight+fmaxf(-change,fminf(change,cmd.right-lastRight));
    lastLeft=left;lastRight=right;speed=(left+right)/2;
    auto prev=pose;
    pose=kyoku::odometry(pose,left*0.01f,right*0.01f,pose.yaw+(right-left)/c.track*0.01f);
    assert(path.safe(pose));
    for(int g=0;g<3;++g) {
      float nx=cosf(gates[g].normal),ny=sinf(gates[g].normal);
      float a=(prev.p.x-gates[g].center.x)*nx+(prev.p.y-gates[g].center.y)*ny;
      float b=(pose.p.x-gates[g].center.x)*nx+(pose.p.y-gates[g].center.y)*ny;
      float lateral=fabsf(-(pose.p.x-gates[g].center.x)*ny+(pose.p.y-gates[g].center.y)*nx);
      if(((a<=0&&b>0)||(a>=0&&b<0))&&lateral<c.gateWidth/2) {assert(g==crossed%3);++crossed;}
    }
  }
  assert(done);assert(crossed==3*c.laps);assert(kyoku::distance(pose.p,finish.p)<10);
  std::printf("PASS %s: %d laps, %d samples, %.0f mm, %d ordered crossings\n",mirror?"mirrored":"normal",c.laps,path.size(),path.length(),crossed);
}
int main(int argc,char **argv) {
  // A stationary first sample must exceed a representative power dead zone;
  // preserve zero output at rest and clamp saturation without winding up.
  float integral=0;
  assert(kyoku::wheelPower(53,0,integral,0.01f)>=kyoku_settings::MinRunPower);
  assert(kyoku::wheelPower(1,0,integral,0.01f)==35);
  for (float target : {1.0f, 20.0f, 53.0f, 130.0f, 220.0f}) {
    for (float measured : {0.0f, 50.0f, 200.0f, 1000.0f}) {
      float wheel_integral = 0;
      const int output = kyoku::wheelPower(target, measured, wheel_integral, 0.01f);
      assert(output == 0 || (output >= 35 && output <= kyoku_settings::MaxPower));
    }
  }
  assert(kyoku::wheelPower(0,0,integral,0.01f)==0&&integral==0);
  for(int i=0;i<500;++i) assert(kyoku::wheelPower(1000,0,integral,0.01f)==kyoku_settings::MaxPower);
  assert(integral==0);
  assert(kyoku::wheelPower(50,1000,integral,0.01f)==0);
  std::puts("PASS minimum power 35, overspeed coasting, zero output and anti-windup");
  auto c=kyoku_settings::config();
  simulate(c,kyoku_settings::Start,kyoku_settings::Gates,kyoku_settings::Finish,false);
  if(argc==2) {
    FILE *f=std::fopen(argv[1],"w");assert(f);
    std::fprintf(f,"x,y,yaw,curvature,s,speed\n");
    for(int i=0;i<path.size();++i) std::fprintf(f,"%f,%f,%f,%f,%f,%f\n",path[i].p.x,path[i].p.y,path[i].yaw,path[i].curvature,path[i].s,path[i].speed);
    std::fclose(f);
  }
  auto start=kyoku_settings::Start,finish=kyoku_settings::Finish;
  start.p.y=-start.p.y;start.yaw=-start.yaw;finish.p.y=-finish.p.y;finish.yaw=-finish.yaw;
  kyoku::Gate gates[3];for(int g=0;g<3;++g) {gates[g]=kyoku_settings::Gates[g];gates[g].center.y=-gates[g].center.y;gates[g].normal=-gates[g].normal;}
  c.minY=-1600;c.maxY=500;
  simulate(c,start,gates,finish,true);
  int progress=0;
  assert(!path.follow({{1500,1500},0},0,progress).valid);
  c.height=c.gateHeight;
  assert(!path.plan(c,start,gates,finish));assert(path.size()==0);
  c=kyoku_settings::config();c.halfWidth=130;
  assert(!path.plan(c,kyoku_settings::Start,kyoku_settings::Gates,kyoku_settings::Finish));
  c=kyoku_settings::config();c.sample=0;
  assert(!path.plan(c,kyoku_settings::Start,kyoku_settings::Gates,kyoku_settings::Finish));
  auto p=kyoku::odometry({{0,0},0},100,100,0);assert(fabsf(p.p.x-100)<0.001f);
  p=kyoku::odometry({{0,0},kyoku::Pi-0.01f},10,10,-kyoku::Pi+0.01f);assert(p.p.x<-9.99f);
  std::puts("PASS invalid geometry, off-path stop, signed odometry and yaw wrap");
}
