#include <iostream>
#include <queue>
#include <memory>


class tst {
public :
 tst(int a):a_(a){
   for (size_t i = 0; i < a; i++)
   {
      m_.push_back(a);
   }
   
 };
 ~tst() {std::cout << "byb: " << a_ << std::endl;};
 int a_;
 std::vector<int> m_;
};
typedef std::shared_ptr<tst> tstPtr;
int main() {

  //  std::queue<tst *> q;
  //  {
  //     //a和b在栈上，一旦出了作用域就会被析构，不过析构后的内存数据还是有可能放在那的，只要这部分数据没有人使用
  //     tst a(1);
  //     tst b(2);
  //     q.push(&a);
  //     q.push(&b);
  //     std::cout << " a =  " << q.front()->m_[0] << std::endl;
  //     q.pop();
  //     std::cout << " b =  " << q.front()->m_[0] << std::endl;
  //  }
  //  tst c(3);
  //  tst d(4);
  //  std::cout << " a =  " << q.front()->m_[0] << std::endl;

  //  std::queue<tst> q2;
  //  q2.push(tst(5));
  // std::cout << "q2 = " << q2.front().m_[4] << std::endl;
  //  q2.pop();
  //  q2.push(tst(6));
  // std::cout << "q2 = " << q2.front().m_[4] << std::endl;
  
  // std::queue<tst*> q3;
  // {
  //   tst* a = new tst(2);
  //   q3.push(a);
  // }
  // std::cout << "a = " << q3.front()->m_[1] << std::endl;
  // //new 出来的必须delete
  // q3.pop();
  //用智能指针的好处就是，当没有变量保存这个指针了会自动析构内存
  std::queue<tstPtr> q4;
  {
    tstPtr a;
    a.reset(new tst(2));
    q4.push(a);
  }
  std::cout << "a = " << q4.back()->m_[1] << std::endl;
  //new 出来的必须delete
  tstPtr b;
  b.reset(new tst(3));
  q4.push(b);
  tstPtr c = q4.front();
  q4.pop();
  std::cout << "b = " << q4.front()->m_[1] << std::endl;

}