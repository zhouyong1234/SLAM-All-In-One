#include "iostream"
#include <list>
class A {
 public:
  A() {
		a_ = std::string("a_mamer");
	}
	 std::string a_;
};
class B {
 public:
  B() = default;
	B(A* a) {
		a_ = a;
		std::cout << "B.a_ = " << a_->a_ << std::endl;
	};
	~B() {
		std::cout << "Deconstuct b" << std::endl;
		//delete a_;//如果不主动delete掉a，那么析构函数默认是不delete掉a的
	}
	A* a_;
};


void func(B* d) {
	A a;
	d->a_ = &a;
	std::cout << "TempValue = " << d->a_->a_ << std::endl;
}

int main() {
	std::cout << "使用标准容器的push-back其实是对变量进行了copy，因此当这个变量不存了，容器中还有它的副本"  << std::endl;
  std::cout << "如果把局部对象的指针传给全局变量的成员，虽然有可能还能得到这个局部变量里面的数值，但是这个内存已经被声明释放了，后续可能出现错误的值" << std::endl;
 	std::list<A> listA;
	 B d;
 {
 A a;
 std::cout << "a_ptr : " << &a << std::endl;
 listA.push_back(a);
 A* c = &(*listA.begin());
 std::cout << "a = " << c->a_ << " address = " << c  << std::endl;
 printf("Before deconstruct B: a = %s\n",a.a_.c_str());
 {
	 B b(&a);
 }
  d.a_ = &a;
 printf("After deconstruct B: a = %s\n",a.a_.c_str());
 }
 std::cout << "a = " << &(*listA.begin()) << std::endl;
 std::cout << "A address : " << d.a_ << std::endl;
 printf("After out A: a = %s\n",d.a_->a_.c_str());
 B e;
 func(&e);
 std::cout << "E address : " << e.a_ << std::endl;
 printf("After out E: a = %s\n",e.a_->a_.c_str());
 B f;
 func(&f);
 std::cout << "E address : " << e.a_ << std::endl;
 printf("After out E: a = %s\n",e.a_->a_.c_str());
}
