#include <iostream>
#include <functional>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>

template <int D>
Eigen::Matrix<double,D,1> RK4 (Eigen::Matrix<double,D,1> x, double dt, std::function<Eigen::Matrix<double,D,1>(const Eigen::Matrix<double,D,1>&)> f) {
  Eigen::Matrix<double,D,1> k1 = f(x)*dt;
  Eigen::Matrix<double,D,1> k2 = f(x + 0.5*k1)*dt;
  Eigen::Matrix<double,D,1> k3 = f(x + 0.5*k2)*dt;
  Eigen::Matrix<double,D,1> k4 = f(x + k3)*dt;
  return x + (k1 + 2.*k2 + 2.*k3 + k4)/6.;
}


Eigen::Matrix<double,3,1> lorenz(const Eigen::Matrix<double,3,1>& x) {

  double sigma = 10.;
  double rho = 28.;
  double beta = 8./3.;

  Eigen::Matrix<double,3,1> dx;
  dx(0) = sigma * (x(1)-x(0));
  dx(1) = x(0) * (rho-x(2)) - x(1);
  dx(2) = x(0)*x(1) - beta * x(2);
  return dx;
}

Eigen::Matrix<double,3,1> roessler(const Eigen::Matrix<double,3,1>& x) {

  double a = 0.2;
  double b = 0.2;
  double c = 5.7;

  Eigen::Matrix<double,3,1> dx;
  dx(0) = - x(1)-x(2);
  dx(1) = x(0) + a*x(1);
  dx(2) = b + x(2)*(x(0)-c);
  return dx;
}

int main(int argc, char** argv) {

  pangolin::CreateWindowAndBind("Main",640,480);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
  );
  const int UI_WIDTH = 180;
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
	
	pangolin::Var<double> scale ("ui.scale", 10.0, 0.001, 100.0);
	pangolin::Var<double> offset ("ui.offset", 0.5, 0.0, 1.0);
	pangolin::Var<bool> colorByXYZ("ui.color by xyz", false, true, true);

	pangolin::Var<int> T ("ui.T", 40000, 10000, 100000);
	pangolin::Var<double> dt ("ui.dt", 0.001, 0.0001, 0.01);
	pangolin::Var<double> x0x ("ui.x0x", 1.0, 0.0, 1.0);
	pangolin::Var<double> x0y ("ui.x0y", 1.0, 0.0, 1.0);
	pangolin::Var<double> x0z ("ui.x0z", 1.0, 0.0, 1.0);

	pangolin::Var<bool> useLorenz("ui.lorenz", false, true, true);
	pangolin::Var<bool> useRoessler("ui.roessler", true, true, true);

	std::vector<Eigen::Vector3d> xs(T);

  // Default hooks for exiting (Esc) and fullscreen (tab).
	size_t frame = 0;
  while( !pangolin::ShouldQuit() )
  {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
  	
		if (frame ==0 || T.GuiChanged() || dt.GuiChanged() 
				|| x0x.GuiChanged() ||  x0y.GuiChanged() ||  x0z.GuiChanged() 
				|| useRoessler.GuiChanged() || useLorenz.GuiChanged()) {
		
			std::function<Eigen::Vector3d(const Eigen::Vector3d&)> system;
			if (useLorenz) {
				system = lorenz;	
			} else if (useRoessler) {
				system = roessler;
			} else {
				system = lorenz;
			}
			xs.clear();
			xs.reserve(T);
			Eigen::Vector3d x(x0x, x0y, x0z);
			for (size_t i=0; i<T; ++i) {
				x = RK4<3>(x, dt, system);
				xs.emplace_back(x);
			}
		}

    // Activate efficiently by object
    d_cam.Activate(s_cam);

    // Render some stuff
		glBegin(GL_POINTS);
		if (colorByXYZ) {
			for (auto& x : xs) {
				glColor3f(x(0)/scale + offset, x(1)/scale + offset, x(2)/scale + offset);
				glVertex3f(x(0),x(1),x(2));
			}
		} else {
			float t = 0.;
			for (auto& x : xs) {
				glColor3f(t/(float)T,1.-t/(float)T, 1.);
				glVertex3f(x(0),x(1),x(2));
				t++;
			}
		}
		glEnd();

    // Swap frames and Process Events
    pangolin::FinishFrame();
		frame ++;
  }


  return 0;
}
