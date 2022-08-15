# 线性dp

## 目录

<div align="center">
 
   | [1-10]| [11-20] | [21-30]| [31-40]|
   | :---: | :---: |  :---: |   :---: | 
   |[1](#DP1)|[11](#DP11)| [21](#DP21)| [31](#DP31)| 
   |[2](#DP2)|[12](#DP12)| [22](#DP22)| [32](#DP32)| 
   |[3](#DP3)|[13](#DP13)| [23](#DP23)| [33](#DP33)| 
   |[4](#DP4)|[14](#DP14)| [24](#DP24)| 
   |[5](#DP5)|[15](#DP15)| [25](#DP25)| 
   |[6](#DP6)|[16](#DP16)| [26](#DP26)| 
   |[7](#DP7)|[17](#DP17)| [27](#DP27)| 
   |[8](#DP8)|[18](#DP18)| [28](#DP28)| 
   |[9](#DP9)|[19](#DP19)| [29](#DP29)| 
   |[10](#DP10)|[20](#DP20)| [30](#3DP0)| 

 
 </div>

- [DP1](#DP1)
- [DP2](#DP2)
- [DP3](#DP3)
- [DP4](#DP4)
- [DP5](#DP5)
- [DP6](#DP6)
- [DP7](#DP7)
- [DP8](#DP8)
- [DP9](#DP9)
- [DP10](#DP10)
- [DP11](#DP11)
- [DP12](#DP12)
- [DP13](#DP13)
- [DP14](#DP14)
- [DP15](#DP15)
- [DP16](#DP16)
- [DP17](#DP17)
- [DP18](#DP18)
- [DP19](#DP19)
- [DP20](#DP20)
- [DP21](#DP21)
- [DP22](#DP22)
- [DP23](#DP23)
- [DP24](#DP24)
- [DP25](#DP25)
- [DP26](#DP26)
- [DP27](#DP27)
- [DP28](#DP28)
- [DP29](#DP29)
- [DP30](#DP30)
- [DP31](#DP31)
- [DP32](#DP32)
- [DP33](#DP33)


### DP1
* DP1 斐波那契数列

描述
```
大家都知道斐波那契数列，现在要求输入一个正整数 n ，请你输出斐波那契数列的第 n 项。
斐波那契数列是一个满足
```

$$
fib(x)=\left\{\begin{matrix}
1 &x=1,2  \\
fib(x-1)+fib(x-2) &x>2  \\
\end{matrix}\right.
$$


 ```
  的数列
数据范围：1≤n≤40
要求：空间复杂度 O(1)，时间复杂度 O(n) ，本题也有时间复杂度 O(logn) 的解法
```
<!-- ![img]() -->
```cpp
#include<bits/stdc++.h>
using namespace std;

int main(){
    int num1 = 1,num2 = 1;
    int temp;
    int n;
    cin>>n;
    if(n<3){
        cout<<"1"<<endl;
        return 0;
    }
    for(int i=3;i<=n;i++){
        temp  = num2;
        num2 = num1+num2;
        num1 = temp;
    }
    cout<<num2<<endl;
    return 0;
}

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP1
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP2
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP3
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP3
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP3
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


### DP3
* DP1 dosomething

描述
```

```
<!-- ![img]() -->
```cpp

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


