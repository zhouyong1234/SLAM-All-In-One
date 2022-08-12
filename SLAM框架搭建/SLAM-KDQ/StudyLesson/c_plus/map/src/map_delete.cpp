#include <iostream>
#include <map>
int main() {
 std::map<int,int> m1;
 m1[1] = 1;
 m1[2] = 2;
 std::cout << m1[1] << " " << m1[2] << std::endl;
 m1.erase(1);
 //KDQ: erase not exist key, that is ok 
 m1.erase(3);
 std::cout << "map size = " << m1.size() << std::endl;
}

