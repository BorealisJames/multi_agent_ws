#ifndef CONSTANTS_MATH_H
#define CONSTANTS_MATH_H

#include <stdarg.h>
#include <limits>
#include <array>
#include <complex>

#include <geometry_msgs/Quaternion.h>

namespace Common
{

namespace Math
{
    // Returns the maximum of <count> variables of identical type. IMPORTANT: count must be exactly the number of variables passed to this function.
    template <class T>
    T max(size_t count, T var1, ...)
    {
        va_list ap;
        va_start(ap,var1);
        T maxVal = 0.0;
        T val = 0.0;
        for(size_t i=1; i<count; i++)
        {
            val = va_arg(ap, T);
            if (val > maxVal)
            {
                maxVal = val;
            }
        }
        va_end(ap);
        
        return maxVal;
    }

    // Returns the sum of <count> variables of identical type. IMPORTANT: count must be exactly the number of variables passed to this function.
    template <class T>
    T sum(size_t count, T var1, ...)
    {
        va_list ap;
        va_start(ap,var1);
        T sum = var1;
        for(size_t i=1; i<count; i++)
        {
            sum += va_arg(ap, T);
        }
        va_end(ap);
        
        return sum;
    }

    template <class T, size_t n>
    void M_identity(std::array<std::array<T,n>,n>& M)
    {
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < n; c++)
            {
                if (r==c)
                {
                    M[r][c] = 1;
                }
                else
                {
                    M[r][c] = 0;
                }
            }
        }
    }

    // Returns the sum of two matrices: A + B
    template <class T, size_t n, size_t m>
    std::array<std::array<T,m>,n> M_sum(const std::array<std::array<T,m>,n>& A, const std::array<std::array<T,m>,n>& B)
    {
        std::array<std::array<T,m>,n> result;
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < m; c++)
            {
                result[r][c] = A[r][c] + B[r][c];
            }
        }
        return result;    
    }

    // Returns the difference of two matrices: A - B
    template <class T, size_t n, size_t m>
    std::array<std::array<T,m>,n> M_diff(const std::array<std::array<T,m>,n>& A, const std::array<std::array<T,m>,n>& B)
    {
        std::array<std::array<T,m>,n> result;
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < m; c++)
            {
                result[r][c] = A[r][c] - B[r][c];
            }
        }
        return result;    
    }

    // Returns the vector product A*B_transpose where A and B are both column vectors
    template <class T, size_t n, size_t m>
    std::array<std::array<T,m>,n> V_product_ABT(const std::array<T,n>& A, const std::array<T,m>& B)
    {
        std::array<std::array<T,m>,n> result;
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < m; c++)
            {
                result[r][c] = A[r]*B[c];
            }
        }
        return result;    
    }

    // Returns the scalar product s*M where s is the scalar and M is a matrix
    template <class T, size_t n, size_t m>
    std::array<std::array<T,m>,n> S_product(const T& s, const std::array<std::array<T,m>,n>& M)
    {
        std::array<std::array<T,m>,n> result;
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < m; c++)
            {
                result[r][c] = s*M[r][c];
            }
        }
        return result;    
    }

    // Returns the matrix-vector product M*V where M is the matrix and V is a column vector
    template <class T, size_t n, size_t m>
    std::array<T,n> MV_product(const std::array<std::array<T,m>,n>& M, const std::array<T,m>& V)
    {
        std::array<T,n> result = {0};
        for (size_t r = 0; r < n; r++)
        {
            for (size_t c = 0; c < m; c++)
            {
                result[r] += M[r][c]*V[c];
            }
        }
        return result;    
    }

    static double wrap(const double& angle)
    {
        double result = angle;
        while(result > M_PI)
            result -= 2.0*M_PI;
        while(result < -M_PI)
            result += 2.0*M_PI;
        return result;
    }

    static geometry_msgs::Quaternion normalise(const geometry_msgs::Quaternion& q)
    {
        const double length = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        geometry_msgs::Quaternion q_normed = q;
        if (length > std::numeric_limits<double>::epsilon())
        {
            q_normed.w /= length;
            q_normed.x /= length;
            q_normed.y /= length;
            q_normed.z /= length;
        }
        else
        {
            q_normed.w = 1.0;
            q_normed.x = 0.0;
            q_normed.y = 0.0;
            q_normed.z = 0.0;
        }
        return q_normed;
    }
    
}   // Math

}   // Common

#endif  //CONSTANTS_MATH_H