#ifndef UTILITY_H
#define UTILITY_H

using namespace std;

/**====================================================
* Function to return evenly spaced numbers over a specified interval.
* Input: Start point, end point, steps
* Output: Vector of evenly spaced numbers
*======================================================*/
template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

	std::vector<double> linspaced;

	double start = static_cast<double>(start_in);
	double end = static_cast<double>(end_in);
	double num = static_cast<double>(num_in);

	if (num == 0) { return linspaced; }
	if (num == 1)
	{
		linspaced.push_back(start);
		return linspaced;
	}

	double delta = (end - start) / (num - 1);

	for (int i = 0; i < num - 1; ++i)
	{
		linspaced.push_back(start + delta * i);
	}
	linspaced.push_back(end);

	return linspaced;
}


/**====================================================
* Function to sort an input vector return index w.r.t. original
* Input: Array, Size of the array
* Output: Indexing of the sorted vector
*======================================================*/

template <typename T>
vector<size_t> sort_indexes(vector<T>& v) {

	// initialize original index locations
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	//sort(idx.begin(), idx.end(),
	stable_sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });

	return idx;
}

#endif // UTILITY_H

