# Eigen  

### 动态大小空间Dynamic

编译时大小未知，

typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

typedef Matrix<int, Dynamic, 1> [VectorXi](https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#gaf20e523ca57ee8ef0a945cd4703d2bfd);

Matrix<float, 3, Dynamic>

MatrixXd :is a dynamic-size matrix whose size is currently 0-by-0, and whose array of coefficients hasn't yet been allocated at all.

### Resizing  

The resize() method is a no-operation if the actual matrix size doesn't change; otherwise it is destructive: the values of the coefficients may change. If you want a conservative variant of resize() which does not change the coefficients, use **conservativeResize()**, see this page for more details.

**conservativeResize():**

All these methods are still available on fixed-size matrices, for the  sake of API uniformity. Of course, you can't actually resize a  fixed-size matrix. Trying to change a fixed size to an actually  different value will trigger an assertion failure; but the following  code is legal:

### Assignment and resizing

Assignment is the action of copying a matrix into another, using operator=. Eigen resizes the matrix on the left-hand side automatically so that it matches the size of the matrix on the right-hand size.

MatrixXf a(2,2);  

MatrixXf b(3,3);

a = b;

```c
a is of size 2x2
a is now of size 3x3
```



### Fixed vs. Dynamic size  

在可能的情况下，对非常小的尺寸使用固定尺寸，对较大的尺寸或必须使用的尺寸使用动态尺寸。对于较小的尺寸，尤其是小于（大约）16 **float mymatrix[16]; **，使用固定尺寸对性能非常有益，因为它允许Eigen避免动态内存分配并展开循环

MatrixXf mymatrix(rows,columns); 

 amounts to doing 

float *mymatrix = new float[rows*columns]; 

### Optional template parameters  

Matrix类有六个模板参数，但到目前为止，我们只讨论了前三个。其余三个参数是可选的.

Matrix<typename Scalar,

​       int RowsAtCompileTime,

​       int ColsAtCompileTime,

​       int Options = 0,

​       int MaxRowsAtCompileTime = RowsAtCompileTime,

​       int MaxColsAtCompileTime = ColsAtCompileTime>

RowMajor。它指定此类型的矩阵使用行主存储顺序；默认情况下，存储顺序为column-major。

Matrix<float, 3, 3, RowMajor>

MaxRowsAtCompileTime和MaxColsAtCompleTime在您想要指定的情况下很有用，即使在编译时矩阵的确切大小未知，但在编译时固定的上限是已知的。您可能想要这样做的最大原因是避免动态内存分配。例如，以下矩阵类型使用12个浮点的普通数组，而不进行动态内存分配：

Matrix<float, Dynamic, Dynamic, 0, 3, 4>

### Convenience typedefs  

- `MatrixNt` for `Matrix<type, N, N>`. For example, `MatrixXi` for `Matrix<int, Dynamic, Dynamic>`. 
- `MatrixXNt` for `Matrix<type, Dynamic, N>`. For example, `MatrixX3i` for `Matrix<int, Dynamic, 3>`. 
- `MatrixNXt` for `Matrix<type, N, Dynamic>`. For example, `Matrix4Xd` for `Matrix<d, 4, Dynamic>`. 
- `VectorNt` for `Matrix<type, N, 1>`. For example, `Vector2f` for `Matrix<float, 2, 1>`. 
- `RowVectorNt` for `Matrix<type, 1, N>`. For example, `RowVector3d` for `Matrix<double, 1, 3>`.

## Matrix and vector arithmetic  

### Addition and subtraction

矩阵的计算

- binary operator + as in `a+b` 
- binary operator - as in `a-b` 
- unary operator - as in `-a` 
- compound operator += as in `a+=b` 
- compound operator -= as in `a-=b` 

- binary operator * as in `matrix*scalar` 
- binary operator * as in `scalar*matrix` 
- binary operator / as in `matrix/scalar` 
- compound operator *= as in `matrix*=scalar` 
- compound operator /= as in `matrix/=scalar` 

## Transposition and conjugation

Matrix2i a; 

a << 1, 2, 3, 4;

cout << "Here is the matrix a:\n" << a << endl;

a = a.transpose(); // **!!! do NOT do this !!!**

**a.transposeInPlace();**

## Matrix-matrix and matrix-vector multiplication

- binary operator * as in `a*b` 

- compound operator *= as in `a*=b` (this multiplies on the right: `a*=b` is equivalent to `a = a*b`)

- v.dot(w) 点成

- v.cross(w) 叉乘

  
