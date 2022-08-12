#include <iostream>
#include <memory>


class tst {
public :
 tst(int a):a_(a){};
 int a_;
};

int main() {
  tst* normal_ptr;
  tst* single_ptr;
  std::shared_ptr<tst> share_ptr;
  tst* pp;
  {
    std::shared_ptr<tst> p(new tst(100));
    share_ptr = p;
    std::cout << share_ptr->a_ << std::endl;
  }
  std::cout << share_ptr->a_ << std::endl;
  

}