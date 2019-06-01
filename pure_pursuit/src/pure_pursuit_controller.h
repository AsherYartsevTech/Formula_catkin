#ifndef pure_pursuit_controller_h
#define pure_pursuit_controller_h

#include <vector>
#include <cmath>


using std::vector;
using std::pair;

class PurePursuitController
{
private:
	double ld_time;
    double max_velocity;
	double max_accel;
	double length;
	double min_ld;


    vector<pair<double,double>> waypoints;
    //double curr_waypoint;
    int wp_idx;
    
    double dist_between_two_points(const pair<double,double>& p1,
                                   const pair<double,double>& p2)
    {
        double dist_dx = (p1.first - p2.first);
        double dist_dy = (p1.second - p2.second);
        return sqrt(dist_dx*dist_dx + dist_dy*dist_dy);
    }
    
    double calc_slope(const pair<double,double>& begin,
                      const pair<double,double>& end,
                      bool& slope_is_inf)
    {
        double dy = end.second - begin.second;
        double dx = end.first - begin.first;
        if (dx == 0) {
            slope_is_inf = true;
            return 0;
        }
        return (dy)/(dx);
    }
    
    
    pair<double,double> get_polynomial_roots(double a, double b, double c)
    {
        double disc = sqrt(pow(b,2) - (4 * a * c));
        double x1 = ((-b) + disc)/(2 * a);
        double x2 = ((-b) - disc)/(2 * a);
        
        return std::make_pair(x1, x2);
    }
    
public:
    
    // C'tor
    PurePursuitController(double ld_time, double max_velocity, double max_accel, double length, double min_ld, vector<pair<double,double>> waypoints);

    // D'tor
    ~PurePursuitController();

    /*
    *   Control step that gets the cureent pose and calculates the
    *   car inputs.
    *   Returns the lateral error.
    */
    control_step(const vector<double>& Pose, double& velocity,
                       double& alpha)
   {
        double lat_error;
	
      

        pair<double,double> nearest_point;
        pair<double,double> car_pos = std::make_pair(Pose[0], Pose[1]);
        find_nearest_point_on_path(car_pos, nearest_point);
        double err_x = nearest_point.first - car_pos.first;
        double err_y = nearest_point.second - car_pos.second;
        lat_error = sqrt(err_x*err_x + err_y*err_y);

        alpha = calc_alpha(Pose, double& velocity);
        if (alpha > M_PI) {
            alpha = -2 * M_PI - alpha;
        } else if (alpha < - M_PI) {
            alpha = 2 * M_PI + alpha;
        }

    
        return lat_error;
   }

    double calc_alpha(const vector<double>& Pose, double& velocity)
    {
		
        pair<double,double> ld_point;
        pair<double,double> position = std::make_pair(Pose[0], Pose[1]);
        calc_ld_point(position, ld_point, velocity);
        
        double dx = (ld_point.first - Pose[0]);
        double dy = (ld_point.second - Pose[1]);
		double ld_dist = sqrt(dx ^ 2 + dy ^ 2);
        double beta = acos((dx)/(sqrt(dx*dx+dy*dy)));

        if (dy < 0) {
            beta = 2 * M_PI - beta;
        }
		beta = beta - Pose[2];
		double alpha = atan(2 * (this->length)*sin(beta) / ld_dist);
		if (alpha == 0) {
			velocity = this->max_velocity;
		}
		else {
			double r = 0.5*(ld_dist / sin(alpha));
			velocity = sqrt(this->max_accel*r);
			if (velocity > this->max_velocity) {
				velocity = this->max_velocity;
			}
		}

        return (alpha);
    }

    void calc_ld_point(const pair<double,double> car,
                       pair<double,double>& ld_point, velocity)
    {
		double ld_dist = std::max ((this->ld_time)*velocity , this->min_ld) ;
        int next_wp_idx = (this->wp_idx + 1) % this->waypoints.size();
        pair<double,double> next_wp = this->waypoints[next_wp_idx];
        
        double dist = dist_between_two_points(next_wp, car);
        
        while (dist < this->ld) {
            this->wp_idx++;
            this->wp_idx %= this->waypoints.size();
            
            next_wp_idx = (this->wp_idx + 1) % this->waypoints.size();
            next_wp = this->waypoints[next_wp_idx];
            dist = dist_between_two_points(next_wp, car);
        }
        
        pair<double,double> last_wp = this->waypoints[wp_idx];
        bool slope_is_inf=false;
        double m = calc_slope(last_wp, next_wp, slope_is_inf);
        double a=0, b=0, c=0, x0=0, y0=0;
        
        if (slope_is_inf) {
            x0 = last_wp.first;
            a = 1;
            b = (-2) * car.second;
            c = (pow(car.first-x0, 2) + pow(car.second,2) - pow(ld_dist,2));
        } else {
            y0 = last_wp.second - (m * last_wp.first);
            a = 1 + pow(m,2);
            b = (-2) * car.first + 2 * m * (y0-car.second);
            c = (pow(car.first,2) + pow(y0-car.second,2) - pow(ld_dist,2));
        }
        
        pair<double,double> roots = get_polynomial_roots(a,b,c);
        pair<double,double> point1, point2;
        
        if (slope_is_inf) {
            point1 = std::make_pair(last_wp.first, roots.first);
            point2 = std::make_pair(last_wp.first, roots.second);
        } else {
            point1 = std::make_pair(roots.first, m * roots.first + y0);
            point2 = std::make_pair(roots.second, m * roots.second + y0);
        }
        
        double d1 = dist_between_two_points(next_wp, point1);
        double d2 = dist_between_two_points(next_wp, point2);
        
        if (d1 < d2) {
            ld_point = point1;
        } else {
            ld_point = point2;
        }
    }
    
    void find_nearest_point_on_path(const pair<double,double> car_cord,
                                    pair<double,double>& nearest_point)
    {
        pair<double,double> last_wp = this->waypoints[wp_idx];
        int next_wp_idx = (this->wp_idx + 1) % this->waypoints.size();
        pair<double,double> next_wp = this->waypoints[next_wp_idx];
        bool slope_is_inf = false;
        double m = this->calc_slope(last_wp, next_wp, slope_is_inf);
        
        if (slope_is_inf) {
            nearest_point.first = last_wp.first;
            nearest_point.second = car_cord.second;
        } else if (m == 0) {
            nearest_point.first = car_cord.first;
            nearest_point.second = last_wp.second;
        } else {
            double y0_ort = car_cord.second + (1/m)*car_cord.first;
            double y0 = last_wp.second - m*last_wp.first;
            
            nearest_point.first = ((m/(pow(m,2)+1))*(y0_ort-y0));
            nearest_point.second = (m * nearest_point.first) + y0;
        }
        
    }


};

PurePursuitController::PurePursuitController(double ld_time , double velocity,
 vector<pair<double,double>> waypoints) : ld(ld), velocity(velocity), 
 waypoints(waypoints), wp_idx(0)
{
}

PurePursuitController::~PurePursuitController()
{
}

#endif /* pure_pursuit_controller_h */
