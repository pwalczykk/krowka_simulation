#ifndef TEMPLATE_VARIABLE_HPP_
#define TEMPLATE_VARIABLE_HPP_

#include <cstring>
#include <cmath>

template <typename data_type>
class MatlabVariable{
public:
    data_type data;

public:

    MatlabVariable(){
    }

    MatlabVariable(data_type data){
        this->data = data;
    }

    ~MatlabVariable(){};

// Minus operator

    MatlabVariable operator-(){
        MatlabVariable result (-1 * this->data);
        return result;
    }
};

// Class & Class argument operators
    template <typename data_type>
    MatlabVariable <data_type> operator+(const MatlabVariable<data_type>& mv1, const MatlabVariable<data_type>& mv2){
        MatlabVariable <data_type> result (mv1.data + mv2.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator-(const MatlabVariable<data_type>& mv1, const MatlabVariable<data_type>& mv2){
        MatlabVariable <data_type> result (mv1.data - mv2.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator*(const MatlabVariable<data_type>& mv1, const MatlabVariable<data_type>& mv2){
        MatlabVariable <data_type> result (mv1.data * mv2.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator/(const MatlabVariable<data_type>& mv1, const MatlabVariable<data_type>& mv2){
        MatlabVariable <data_type> result (mv1.data / mv2.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator->*(const MatlabVariable<data_type>& mv1, const MatlabVariable<data_type>& mv2){
        MatlabVariable <data_type> result ( pow(mv1.data, mv2.data) );
        return result;
    }


// Class & Int argument operators
    template <typename data_type>
    MatlabVariable <data_type> operator+(const MatlabVariable<data_type>& mv, const int& dt){
        MatlabVariable <data_type> result (mv.data + (data_type)dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator-(const MatlabVariable<data_type>& mv, const int& dt){
        MatlabVariable <data_type> result (mv.data - (data_type)dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator*(const MatlabVariable<data_type>& mv, const int& dt){
        MatlabVariable <data_type> result (mv.data * (data_type)dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator/(const MatlabVariable<data_type>& mv, const int& dt){
        MatlabVariable <data_type> result (mv.data / (data_type)dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator->*(const MatlabVariable<data_type>& mv, const int& dt){
        MatlabVariable <data_type> result ( pow(mv.data, (data_type)dt) );
        return result;
    }

// Int & Class argument operators
    template <typename data_type>
    MatlabVariable <data_type> operator+(const int& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ((data_type)dt + mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator-(const int& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ((data_type)dt - mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator*(const int& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ((data_type)dt * mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator/(const int& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ((data_type)dt / mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator->*(const int& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ( pow((data_type)dt, mv.data) );
        return result;
    }

// Class & Double argument operators
    template <typename data_type>
    MatlabVariable <data_type> operator+(const MatlabVariable<data_type>& mv, const double& dt){
        MatlabVariable <data_type> result (mv.data + dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator-(const MatlabVariable<data_type>& mv, const double& dt){
        MatlabVariable <data_type> result (mv.data - dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator*(const MatlabVariable<data_type>& mv, const double& dt){
        MatlabVariable <data_type> result (mv.data * dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator/(const MatlabVariable<data_type>& mv, const double& dt){
        MatlabVariable <data_type> result (mv.data / dt);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator->*(const MatlabVariable<data_type>& mv, const double& dt){
        MatlabVariable <data_type> result ( pow(mv.data, dt) );
        return result;
    }

// Double & Class argument operators
    template <typename data_type>
    MatlabVariable <data_type> operator+(const double& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result (dt + mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator-(const double& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result (dt - mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator*(const double& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result (dt * mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator/(const double& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result (dt / mv.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> operator->*(const double& dt, const MatlabVariable<data_type>& mv){
        MatlabVariable <data_type> result ( pow(dt, mv.data) );
        return result;
    }

// Trigonometric functions
    template <typename data_type>
    MatlabVariable <data_type> sin(MatlabVariable <data_type> input){
        MatlabVariable <data_type> result;
        result.data = sin(input.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> cos(MatlabVariable <data_type> input){
        MatlabVariable <data_type> result;
        result.data = cos(input.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> tan(MatlabVariable <data_type> input){
        MatlabVariable <data_type> result;
        result.data = tan(input.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> atan(MatlabVariable <data_type> input){
        MatlabVariable <data_type> result;
        result.data = atan(input.data);
        return result;
    }

    template <typename data_type>
    MatlabVariable <data_type> atan2(MatlabVariable <data_type> input1, MatlabVariable <data_type> input2){
        MatlabVariable <data_type> result;
        result.data = atan2(input1.data, input2.data);
        return result;
    }


#endif
