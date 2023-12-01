
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main()
{
	ofstream file;
	file.open("test.csv");

	if(!file.is_open())
    {
        std::cout << " not check" << std::endl;
    }

	file << "test" << " " << "another test" << std::endl;




	// vector<vector<string>> content;
	// vector<string> row;
	// string line, word;

	// fstream file ("ground_truth.csv", ios::in);
	// if(file.is_open())
	// {
	// 	while(getline(file, line))
	// 	{
	// 		row.clear();

	// 		stringstream str(line);

	// 		while(getline(str, word, ' '))
	// 			row.push_back(word);
	// 		content.push_back(row);
	// 	}
	// }
	// else
	// 	cout<<"Could not open the file\n";

	// // for(int i=0;i<content.size();i++)
	// // {
	// // 	for(int j=0;j<content[i].size();j++)
	// // 	{
	// // 		cout<<content[i][j]<<", ";
	// // 	}
	// // 	cout<<"\n";
	// // }

    // std::cout << content.size() << std::endl;

	return 0;
}

