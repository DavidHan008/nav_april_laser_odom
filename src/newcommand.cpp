#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Pose2D.h>
#include<nav_april_laser_odom/command.h>
#include<vector>
struct planpoint
{
    double x;
    double y;
    double theta;
};
bool run_state;
double x_start,y_start,theta_start;//TODO：theta_start是已经规范化的角度，不是实时的角度
//起始点的网格坐标系（以世界坐标系为参考系）
int x_startworldGrid,y_startworldGrid,theta_startworldGrid;
//起始点的网格坐标系（以机器人坐标系为参考系）
int x_startGrid,y_startGrid,theta_startGrid;

double x_goal,y_goal,theta_goal;
//目标点的网格坐标（以世界坐标系为参考系）
int x_goalworldGrid,y_goalworldGrid,theta_goalworldGrid;
//目标点的网格坐标（以机器人坐标系为参考系）
int x_goalGrid,y_goalGrid,theta_goalGrid;
double normalTheta(double theta)
{
    if(theta>M_PI)
    {
        theta-=2*M_PI;
    }
    else if(theta<-M_PI)
    {
        theta+=2*M_PI;
    }
    return theta;
}
int func(double x)
{
    if(x>0.0)
        return (int)(x+0.5);
    else
        return (int)(x-0.5);
}
int getPosition(int dx,int dy)
{
    int pose_flag;
    //0：位置重合；1：右上方；2：左上方；3：左下方；4：右下方；5：正上方；6：正左方；7：正下方；8：正右方；
    if(dx>0)//目标点在当前位置上方
    {
        if(dy>0)
        {
            pose_flag=2;//左上方
            ROS_INFO("LEFT UP");
        }
        else if(dy<0)
        {
            pose_flag=1;//右上方
            ROS_INFO("RIGHT UP");
        }
        else
        {
            pose_flag=5;//正上方
            ROS_INFO("UP UP");
        }
    }
    else if(dx<0)//目标点在当前位置下方
    {
        if(dy>0)
        {
            pose_flag=3;//左下方
            ROS_INFO("LEFT DOWN");
        }
        else if(dy<0)
        {
            pose_flag=4;//右下方
            ROS_INFO("RIGHT DOWN");
        }
        else
        {
            pose_flag=7;//正下方
            ROS_INFO("DOWN DOWN");
        }
    }
    else//目标点在当前位置的水平方向
    {
        if(dy>0)
        {
            pose_flag=6;//正左方
            ROS_INFO("LEFT LEFT");
        }
        else if(dy<0)
        {
            pose_flag=8;//正右方
            ROS_INFO("RIGHT RIGHT");
        }
        else
        {
            pose_flag=0;//重合
            ROS_INFO("SAME");
        }
    }
    return pose_flag;
}
int getHead()
{
    int head;//0：朝上；1：朝左；2：朝下；3：朝右
    if(-0.1745<theta_start&&theta_start<0.1745)//0 degree
    {
        head=0;
        theta_start=0;
        x_startGrid=func(x_start/1.2);
        y_startGrid=func(y_start/1.2);
        x_goalGrid=func(x_goal/1.2);
        y_goalGrid=func(y_goal/1.2);
    }
    else if(1.3963<theta_start&&theta_start<1.7453)//90 degree
    {
        head=1;
        theta_start=M_PI/2;
        x_startGrid=func(y_start/1.2);
        y_startGrid=-func(x_start/1.2);
        x_goalGrid=func(y_goal/1.2);
        y_goalGrid=-func(x_goal/1.2);
    }
    else if(2.967<theta_start||theta_start<-2.967)//180 degree
    {
        head=2;
        theta_start=M_PI;
        x_startGrid=-func(x_start/1.2);
        y_startGrid=-func(y_start/1.2);
        x_goalGrid=-func(x_goal/1.2);
        y_goalGrid=-func(y_goal/1.2);
    }
    else if(-1.7453<theta_start&&theta_start<-1.3963)//-90 degree
    {
        head=3;
        theta_start=-M_PI/2;
        x_startGrid=-func(y_start/1.2);
        y_startGrid=func(x_start/1.2);
        x_goalGrid=-func(y_goal/1.2);
        y_goalGrid=func(x_goal/1.2);
    }
    return head;
}
/*
 int move[7]={3 ,2,3,2,3,2,3};
double plan[7][3]={1.2,0,0,
                               1.2,0,-M_PI/2,
                               1.2,-1.2,-M_PI/2,
                               1.2,-1.2,-M_PI,
                               0,-1.2,-M_PI,
                               0,-1.2,-M_PI/2,
                               0,-1.8,-M_PI/2};

double plan[7][3]={1.2,0,0,
                               1.2,0,-M_PI/2,
                               1.2,-1.2,-M_PI/2,
                               1.2,-1.2,-M_PI,
                               0,-1.2,-M_PI,
                               0,-1.2,M_PI/2,
                               0,0,M_PI/2};
*/
std::vector<int> move;
std::vector<planpoint > plan;
int k=0;//记录需要的步骤数量
int i=0;
ros::Publisher command;
//std_msgs::String path;
nav_april_laser_odom::command path;
void reachCB(const geometry_msgs::Pose2D::ConstPtr msg)
{
    ROS_INFO("GET THE START POSE");
    x_start=msg->x;
    y_start=msg->y;
    theta_start=msg->theta;
    if(!move.empty()&&k!=0)
    {
        if(i<k)
        {
            ROS_INFO("GET THE PATH");
            path.command=move[i];
            path.pose.x=plan[i].x;
            path.pose.y=plan[i].y;
            path.pose.theta=plan[i].theta;
            command.publish(path);
            i++;
        }
        else
        {
            ROS_INFO("I AM IN THE END OF PLAN!");
            run_state=true;
            move.clear();
            plan.clear();
            i=0;
        }

    }
/*
    if(i<7)
    {
        ROS_INFO("GET THE PATH");
        //path.data=plans[i];
        path.command=move[i];
        path.pose.x=plan[i][0];
        path.pose.y=plan[i][1];
        path.pose.theta=plan[i][2];
        command.publish(path);
        i++;
    }
    else
    {
        ROS_INFO("I AM IN THE END OF PLAN!");
        run_state=true;
    }
    */
}
void setPlan(int x_Grid,int y_Grid,double theta)
{
    planpoint point;
    point.x=x_Grid*1.2;
    point.y=y_Grid*1.2;
    point.theta=theta;
    plan.push_back(point);
}
void go_upPlan(int dx,int x_From,int y_From,double theta_From)
{
    for(int i=1;i<=dx;i++)
    {
        k++;
        move.push_back(3);
        setPlan(x_From+i,y_From,theta_From);
        k++;
        move.push_back(2);
        setPlan(x_From+i,y_From,theta_From);
    }
}
void go_downPlan(int dx,int x_From,int y_From,double theta_From)
{
    for(int i=1;i<=-dx;i++)
    {
        k++;
        move.push_back(3);
        setPlan(x_From-i,y_From,theta_From);
        k++;
        move.push_back(2);
        setPlan(x_From-i,y_From,theta_From);
    }
}
void go_leftPlan(int dy,int x_From,int y_From,double theta_From)
{
    for(int i=1;i<=dy;i++)
    {
        k++;
        move.push_back(3);
        setPlan(x_From,y_From+i,theta_From);
        k++;
        move.push_back(2);
        setPlan(x_From,y_From+i,theta_From);
    }
}
void go_rightPlan(int dy,int x_From,int y_From,double theta_From)
{
    for(int i=1;i<=-dy;i++)
    {
        k++;
        move.push_back(3);
        setPlan(x_From,y_From-i,theta_From);
        k++;
        move.push_back(2);
        setPlan(x_From,y_From-i,theta_From);
    }
}
void makePlan(int dx,int dy,int pose_flag,int head)
{
    ROS_INFO("head:%d,pose_falg:%d",head,pose_flag);
    ROS_INFO("dx:%d,dy:%d",dx,dy);
    move.clear();
    plan.clear();
    k=0;
    double tempTheta=0;
    switch (pose_flag) {
    case 0://位置重合，只需要旋转
    {
        double dtheta=normalTheta(theta_goal-theta_start);
        if(dtheta==0)
        {
            ROS_INFO("i am in the goal palce, no need moving");
        }
        else
        {
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 1://右上方
    {
        ROS_INFO("YES! RIGHT UP");
        if(head==0)
        {
            //先向上走，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //向右转
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==1)
        {
            //先向左走，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //向右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start-M_PI/2);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start-M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==2)
        {
            //先向下走，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //向右转
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else
        {
            //先向右走，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //向右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start-M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start-M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 2://左上方
    {
        if(head==0)
        {
            //先向上走，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //向左转
            k++;
            move.push_back(1);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==1)
        {
            //先向左走，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //向左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start+M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start+M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==2)
        {
            //先向下走，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //向左转
            k++;
            move.push_back(1);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else
        {
            //先向右走，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //向左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start+M_PI/2);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start+M_PI/2);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 3://左下方
    {
        if(head==0)
        {
            //先左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start+M_PI);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start+M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==1)
        {
            //先左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //左转
            k++;
            move.push_back(1);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start+M_PI);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start+M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==2)
        {
            //先左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start+M_PI);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start+M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else
        {
            //先左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //左转
            k++;
            move.push_back(1);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start+M_PI);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start+M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 4://右下方
    {
        if(head==0)
        {
            //先右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start-M_PI);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start-M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==1)
        {
            //先右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向上直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //右转
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start-M_PI);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start-M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==2)
        {
            //先右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_goalworldGrid,theta_start-M_PI);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_goalworldGrid,theta_start-M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else
        {
            //先右转
            k++;
            move.push_back(2);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //右转
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_startworldGrid,theta_start-M_PI);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_goalworldGrid,y_startworldGrid,theta_start-M_PI);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 5://正上方
    {
        if(head==0)
        {
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==1)
        {
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else if(head==2)
        {
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
        else
        {
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start);
            //调整角度
            k++;
            move.push_back(2);
            setPlan(x_goalworldGrid,y_goalworldGrid,theta_goal);
        }
    }
        break;
    case 6://正左方
    {
        if(head==0)
        {
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
        }
        else if(head==1)
        {
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
        }
        else if(head==2)
        {
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
        }
        else
        {
            //左转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI/2);
        }
    }
        break;
    case 7://正下方
    {
        if(head==0)
        {
            //转180
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI);
            //向下直行，每个节点调整角度
            go_downPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI);
        }
        else if(head==1)
        {
            //转180
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI);
        }
        else if(head==2)
        {
            //转180
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start+M_PI);
        }
        else
        {
            //转180
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start+M_PI);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start+M_PI);
        }
    }
        break;
    case 8://正右方
    {
        if(head==0)
        {
            //右转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向右直行，每个节点调整角度
            go_rightPlan(dy,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
        }
        else if(head==1)
        {
            //右转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向上直行，每个节点调整角度
            go_upPlan(dx,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
        }
        else if(head==2)
        {
            //右转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向左直行，每个节点调整角度
            go_leftPlan(dy,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
        }
        else
        {
            //右转
            k++;
            move.push_back(1);
            setPlan(x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
            //向下直行，每个节点调整角度
            go_downPlan(dy,x_startworldGrid,y_startworldGrid,theta_start-M_PI/2);
        }
    }
        break;
    default:
        break;
    }
}

void goalCB(const geometry_msgs::Pose2D::ConstPtr goal)
{
    if(run_state==true)
    {
        run_state=false;
        x_goal=goal->x;
        y_goal=goal->y;
        theta_goal=goal->theta;
        //确定机器人位置和目标点位置在世界坐标系的网格坐标
        x_startworldGrid=func(x_start/1.2);
        y_startworldGrid=func(y_start/1.2);
        x_goalworldGrid=func(x_goal/1.2);
        y_goalworldGrid=func(y_goal/1.2);
        //确定车头朝向
        int head;//0：朝上；1：朝左；2：朝下；3：朝右
        head=getHead();
        ROS_INFO("in robot coordinate startX:%d,startY:%d,theta_start:%f",x_startGrid,y_startGrid,theta_start);
        ROS_INFO("goalX:%d,goalY:%d",x_goalGrid,y_goalGrid);
        ROS_INFO("in world coordinate startX:%d,startY:%d",x_startworldGrid,y_startworldGrid);
        ROS_INFO("in world coordinate goalX:%d,goalY:%d",x_goalworldGrid,y_goalworldGrid);
         //确定目标点和当前点相对关系
        int pose_flag;
         //0：位置重合；1：右上方；2：左上方；3：左下方；4：右下方；5：正上方；6：正左方；7：正下方；8：正右方；
        int dx,dy,dtheta;
        dx=x_goalGrid-x_startGrid;
        dy=y_goalGrid-y_startGrid;
        int dx_world,dy_world;
        dx_world=x_goalworldGrid-x_startworldGrid;
        dy_world=y_goalworldGrid-y_startworldGrid;
        //dtheta=theta_goalGrid-theta_startGrid;
        pose_flag=getPosition(dx,dy);
        makePlan(dx_world,dy_world,pose_flag,head);
        std::vector<int>::iterator it;
        for(it=move.begin();it!=move.end();it++)
        {
            std::cout<<"move"<<*it<<',';
        }
        std::cout<<std::endl;
        std::vector<planpoint>::iterator itt;
        for(itt=plan.begin();itt!=plan.end();itt++)
        {
            std::cout<<"move"<<(*itt).x <<','<<(*itt).y<<',' <<(*itt).theta<<std::endl;
        }
    }
    else{
        ROS_WARN("I am running, can not plan now!please wait now~");
    }
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"newcommand");
    ros::NodeHandle nh;
    ros::Subscriber reachflag=nh.subscribe("/reachflag",1,reachCB);
    ros::Subscriber goal_sub=nh.subscribe("goal",1,goalCB);
    command=nh.advertise<nav_april_laser_odom::command>("command",1);
    run_state=true;
    //_goal=2.3;
    //y_goal=-2.44;
    theta_goal=M_PI/2;
    ros::spin();

}



