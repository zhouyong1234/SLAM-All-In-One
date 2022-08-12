#include "marginalization_factor.h"

/**
 * @brief 待边缘化的各个残差块计算残差和雅克比矩阵，同时处理核函数的case
 * 
 */
void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());   // 确定残差的维数

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();  // 确定相关的参数块数目
    raw_jacobians = new double *[block_sizes.size()];   // ceres接口都是double数组，因此这里给雅克比准备数组
    jacobians.resize(block_sizes.size());
    // 这里就是把jacobians每个matrix地址赋给raw_jacobians，然后把raw_jacobians传递给ceres的接口，这样计算结果直接放进了这个matrix
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);    // 雅克比矩阵大小 残差×变量
        raw_jacobians[i] = jacobians[i].data();
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    // 调用各自重载的接口计算残差和雅克比
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);  // 这里实际上结果放在了jacobians

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);
    // 如果有核函数，那么就对残差进行相关调整
    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();  // 获得残差的模
        loss_function->Evaluate(sq_norm, rho);  // rho[0]:核函数这个点的值 rho[1]这个点的导数 rho[2]这个点的二阶导数
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))  // 柯西核p = log(s+1),rho[2]<＝0始终成立，一般核函数二阶导数都是小于0
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }
        // 这里就相当于残差雅克比都乘上sqrt_rho1_，及核函数所在的点的一阶导，基本都是小于1
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    //ROS_WARN("release marginlizationinfo");
    
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete[] it->second;

    for (int i = 0; i < (int)factors.size(); i++)
    {

        delete[] factors[i]->raw_jacobians;
        
        delete factors[i]->cost_function;

        delete factors[i];
    }
}

/**
 * @brief 收集各个残差
 * 
 * @param[in] residual_block_info 
 */
void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);  // 残差块收集起来

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;    // 这个是和该约束相关的参数块
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();   // 各个参数块的大小

    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        // 这里是个map，避免重复添加
        parameter_block_size[reinterpret_cast<long>(addr)] = size;  // 地址->global size
    }
    // 待边缘化的参数块
    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        // 先准备好待边缘化的参数块的map
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

/**
 * @brief 将各个残差块计算残差和雅克比，同时备份所有相关的参数块内容
 * 
 */
void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)
    {
        it->Evaluate(); // 调用这个接口计算各个残差块的残差和雅克比矩阵

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();  // 得到每个残差块的参数块大小
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);    // 得到该参数块的地址
            int size = block_sizes[i];  // 参数块大小
            // 把各个参数块都备份起来，使用map避免重复参数块，之所以备份，是为了后面的状态保留
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
                // 深拷贝
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                parameter_block_data[addr] = data;  // 地址->参数块实际内容的地址
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

/**
 * @brief 分线程构造Ax = b
 * 
 * @param[in] threadsstruct 
 * @return void* 
 */
void* ThreadsConstructA(void* threadsstruct)
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    // 遍历这么多分配过来的任务
    for (auto it : p->sub_factors)
    {
        // 遍历参数块
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])]; // 在大矩阵中的id，也就是落座的位置
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            // 确保是local size
            if (size_i == 7)
                size_i = 6;
            // 之前pre margin 已经算好了各个残差和雅克比，这里取出来
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            // 和本身以及其他雅克比块构造H矩阵
            // i: 当前参数块， j: 另一个参数块
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];    // 在大矩阵中的id，也就是落座的位置
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                // 两个雅克比都取出来了
                if (i == j) // 如果是自己，一块田地
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else    // 和别人，由于对称，两块田地
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    // 第一块田地转置一下
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            // 然后构建g矩阵
            // ? : 为什么不是-JTb
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

/**
 * @brief 边缘化处理，并将结果转换成残差和雅克比的形式
 * 
 */

void MarginalizationInfo::marginalize()
{
    int pos = 0;
    // parameter_block_idx key是各个待边缘化参数块地址 value预设都是0
    for (auto &it : parameter_block_idx)
    {
        it.second = pos;    // 这就是在所有参数中排序的idx，待边缘化的排在前面
        pos += localSize(parameter_block_size[it.first]);   // 因为要进行求导，因此大小时local size，具体一点就是使用李代数
    }

    m = pos;    // 总共待边缘化的参数块总大小（不是个数）
    // 其他参数块
    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;    // 这样每个参数块的大小都能被正确找到
            pos += localSize(it.second);
        }
    }

    n = pos - m;    // 其他参数块的总大小

    //ROS_DEBUG("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());

    TicToc t_summing;
    Eigen::MatrixXd A(pos, pos);    // Ax = b预设大小
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();
    /*
    for (auto it : factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    ROS_INFO("summing up costs %f ms", t_summing.toc());
    */
    //multi thread

    // 往A矩阵和b矩阵中填东西，利用多线程加速
    TicToc t_thread_summing;
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it); // 每个线程均匀分配任务
        i++;
        i = i % NUM_THREADS;
    }
    // 每个线程构造一个A矩阵和b矩阵，最后大家加起来
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;
        // 所以A矩阵和b矩阵大小一样，预设都是0
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        // 多线程访问会带来冲突，因此每个线程备份一下要查询的map
        threadsstruct[i].parameter_block_size = parameter_block_size;   // 大小
        threadsstruct[i].parameter_block_idx = parameter_block_idx; // 索引
        // 产生若干线程
        int ret = pthread_create( &tids[i], NULL, ThreadsConstructA ,(void*)&(threadsstruct[i]));
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for( int i = NUM_THREADS - 1; i >= 0; i--)  
    {
        // 等待各个线程完成各自的任务
        pthread_join( tids[i], NULL );
        // 把各个子模块拼起来，就是最终的Hx = g的矩阵了 
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    //ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());


    //TODO
    // Amm矩阵的构建是为了保证其正定性
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);   // 特征值分解

    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
    // 一个逆矩阵的特征值是原矩阵的倒数，特征向量相同　select类似c++中 ? :运算符
    // 利用特征值取逆来构造其逆矩阵
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m);  // 带边缘化的大小
    Eigen::MatrixXd Amr = A.block(0, m, m, n);  // 对应的四块矩阵
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n); // 剩下的参数
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    // 这个地方根据Ax = b => JT*J = - JT * e
    // 对A做特征值分解 A = V * S * VT,其中Ｓ是特征值构成的对角矩阵
    // 所以J = S^(1/2) * VT , 这样JT * J = (S^(1/2) * VT)T * S^(1/2) * VT = V * S^(1/2)T *  S^(1/2) * VT = V * S * VT(对角矩阵的转置等于其本身)
    // e = -(JT)-1 * b = - (S^-(1/2) * V^-1) * b = - (S^-(1/2) * VT) * b
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    // 对A矩阵取逆
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt(); // 这个求得就是 S^(1/2)，不过这里是向量还不是矩阵
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    // 边缘化为了实现对剩下参数块的约束，为了便于一起优化，就抽象成了残差和雅克比的形式，这样也形成了一种残差约束
    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    //std::cout << A << std::endl
    //          << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx)  // 遍历边缘化相关的每个参数块
    {
        if (it.second >= m) // 如果是留下来的，说明后续会对其形成约束
        {
            keep_block_size.push_back(parameter_block_size[it.first]);  // 留下来的参数块大小 global size
            keep_block_idx.push_back(parameter_block_idx[it.first]);    // 留下来的在原向量中的排序
            keep_block_data.push_back(parameter_block_data[it.first]);  // 边缘化前各个参数块的值的备份
            keep_block_addr.push_back(addr_shift[it.first]); // 对应的新地址
        }
    }
    // 留下来的边缘化后的参数块总大小
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}
/**
 * @brief Construct a new Marginalization Factor:: Marginalization Factor object
 * 边缘化信息结果的构造函数，根据边缘化信息确定参数块总数和大小以及残差维数
 * 
 * @param[in] _marginalization_info 
 */
MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)   // keep_block_size表示上一次边缘化留下来的参数块的大小
    {
        mutable_parameter_block_sizes()->push_back(it); // 调用ceres接口，添加参数块大小信息
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n); // 残差维数就是所有剩余状态量的维数和，这里时local size
};

/**
 * @brief 边缘化结果残差和雅克比的计算
 * 
 * @param[in] parameters 
 * @param[in] residuals 
 * @param[in] jacobians 
 * @return true 
 * @return false 
 */
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}
    int n = marginalization_info->n;    // 上一次边缘化保留的残差块的local size的和,也就是残差维数
    int m = marginalization_info->m;    // 上次边缘化的被margin的残差块总和
    Eigen::VectorXd dx(n);  // 用来存储残差
    // 遍历所有的剩下的有约束的残差块
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;  // idx起点统一到0
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size); // 当前参数块的值
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size); // 当时参数块的值
        if (size != 7)
            dx.segment(idx, size) = x - x0; // 不需要local param的直接做差
        else    // 代表位姿的param
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();    // 位移直接做差
            // 旋转就是李代数做差
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            // 确保实部大于0
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }
    // 更新残差　边缘化后的先验误差 e = e0 + J * dx
    // 个人理解：根据FEJ．雅克比保持不变，但是残差随着优化会变化，因此下面不更新雅克比　只更新残差
    // 可以参考　https://blog.csdn.net/weixin_41394379/article/details/89975386
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    return true;
}
