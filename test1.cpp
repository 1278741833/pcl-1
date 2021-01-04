#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "_freecplus.h"
using namespace std;
const double pi = acos(-1.0);
//定义一种类型表示TXT中xyz
typedef struct TXT_Point_XYZ
{
	double x;
	double y;
	double z;
} TOPOINT_XYZ;

TXT_Point_XYZ txt_points;
vector<TXT_Point_XYZ> my_vTxtPoints;
int invalidcount = 0;
bool SplitBuffer(const char *strBuffer)
{
	CCmdStr CmdStr;
	CmdStr.SplitToCmd(strBuffer, ",", true);
	if (CmdStr.CmdCount() != 4)
	{
		invalidcount++;
		return false;
	}

	double distance = 0;
	double angle = 0;
	//distance angle
	CmdStr.GetValue(1, &distance);
	CmdStr.GetValue(3, &angle);
	if (distance > 10000)
	{+-+
		invalidcount++;
		return false;
	}
	memset(&txt_points, 0, sizeof(struct TXT_Point_XYZ));
	double r = angle * pi / 180;
	txt_points.x = distance * cos(r);
	txt_points.y = distance * sin(r);
	txt_points.z = 0;

	my_vTxtPoints.push_back(txt_points);

	return true;
}

int main()
{
	CDir Dir;
	if (Dir.OpenDir("/home/agv/", "*.CSV", 50000, false, false) == false)
	{
		printf("Dir.OpenDir failed\n");
		return -1;
	}
	int i=1;
	while (true)
	{
		my_vTxtPoints.clear();
		if(Dir.ReadDir()==false) break;

		
		char strFileName[201];
		snprintf(strFileName, 300, "./r2000_%d.pcd",i);
		i++;
		//printf("%s\n",strFileName);

		//读取txt文件
		int num_txt;
		CFile File;

		if (File.Open(Dir.m_FullFileName, "r") == false)
			printf("open file failed\n");
		char strBuffer[301];

		while (true)
		{
			memset(strBuffer, 0, sizeof(strBuffer));

			// 从文件中获取一行记录
			if (File.Fgets(strBuffer, 300, true) == false)
				break;

			UpdateStr(strBuffer, "  ", " ", true); // 把内容中的两个空格替换成一个空格
			//printf("%s\n", strBuffer);

			// 把用' '分隔的记录拆分到结构体中
			if (SplitBuffer(strBuffer) == false)
			{
			//printf("%s拆分失败\n", strBuffer);
				continue;
			}
		}

		num_txt = my_vTxtPoints.size();

		//写入点云数据
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = num_txt;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			cloud->points[i].x = my_vTxtPoints[i].x;
			cloud->points[i].y = my_vTxtPoints[i].y;
			cloud->points[i].z = my_vTxtPoints[i].z;
		}
		pcl::io::savePCDFileASCII(strFileName, *cloud);

		//打印出写入的点
		cout << "_________________________________" << endl;
		/*for (size_t i = 0; i < cloud->points.size(); ++i)
			cout << "    " << cloud->points[i].x
				 << " " << cloud->points[i].y
				 << " " << cloud->points[i].z << endl;*/
		File.CloseAndRemove();
	}
	return 0;
}
