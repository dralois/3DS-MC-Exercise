#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function (check gaussian.cpp for reference)
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& p, const Point2D& q, const Weight& w) :
		_p(p), _q(q), _w(w)
	{
	}

	template<typename T>
	bool operator()(const T* const theta, const T* const tx, const T* const ty, T* residuals) const
	{
		const T& _theta = *theta;
		const T& _tx = *tx;
		const T& _ty = *ty;

		const T cosT = T(cos(_theta));
		const T sinT = T(sin(_theta));

		const T x = ((cosT * T(_p.x) - sinT * T(_p.y)) + _tx) - T(_q.x);
		const T y = ((sinT * T(_p.x) + cosT * T(_p.y)) + _ty) - T(_q.y);

		residuals[0] = T(_w.w) * T(pow(sqrt(pow(x, T(2.0)) + pow(y, T(2.0))), T(2.0)));

		return true;
	}

private:
	Point2D _p;
	Point2D _q;
	Weight _w;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../Data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);

	const std::string file_path_2 = "../Data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);

	const std::string file_path_weights = "../Data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;

	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block (check gaussian.cpp for reference)
	for (size_t i = 0; i < points1.size(); ++i)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
				new RegistrationCostFunction(points1[i], points2[i], weights[i])),
			nullptr,
			&angle, &tx, &ty
		);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
