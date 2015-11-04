/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Utility Chaiscrip API for SMNChai.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cstdlib>
#include <smnchai_api.h>
#include <csvparser.h>  // Read CSV files
#include <smnchai_utils.h>

using namespace SMNChai;

SMNChai::APIUtils::TChaiVector SMNChai::APIUtils::load_csv_into_chai(const std::string& t_file,
                                                                     const std::string& t_format,
                                                                     const std::string& t_delimiter,
                                                                     bool t_header)
{
    // Check delimiter
    const char *delimiter = NULL;
    if (!t_delimiter.empty()) {
        delimiter = t_delimiter.c_str();
    }
    
    // Check format string
    if (t_format.empty()) {
        throw smnchai_exception("The format string must be specified.");
    }
    
    std::size_t pos = t_format.find_first_not_of("dis");
    if (pos != std::string::npos) {
        throw smnchai_exception(std::string("The format string contains an invalid character: ") + t_format[pos]);
    }
    
    auto nfields = t_format.size();
    
    CsvParser *csvparser = CsvParser_new(t_file.c_str(), delimiter, t_header);
    
    if (t_header) {
        CsvRow *header = CsvParser_getHeader(csvparser);
        if (header == NULL) {
            throw smnchai_exception(CsvParser_getErrorMessage(csvparser));
        }
        //char **headerFields = CsvParser_getFields(header);
        //for (auto i = 0 ; i < CsvParser_getNumFields(header) ; i++) {
        //    //printf("TITLE: %s\n", headerFields[i]);
        //}
        
        // Do NOT destroy the headear manually if you plan to destroy the parser later.
        // If you destroy both header and parser, you will get double free runtime error
        // CsvParser_destroy_row(header);
    }
    
    // The Chaiscript's object
    SMNChai::APIUtils::TChaiVector v;  // The entire table
    SMNChai::APIUtils::TChaiVector vr; // Each row
    
    CsvRow *row;
    
    while ((row = CsvParser_getRow(csvparser)) ) {
        char **rowFields = CsvParser_getFields(row);
        
        int n = CsvParser_getNumFields(row);
        if (n > 0) {
            vr.clear();
            for (int i = 0 ; i < n ; ++i) {
                if (i >= nfields) {
                    break;  // from reading the current row
                }
                switch (t_format[i]) {
                    case 'd':  // double value
                        vr.emplace_back(std::strtod(rowFields[i], nullptr));
                        break;
                        
                    case 'i':  // integer value
                        vr.emplace_back(std::strtol(rowFields[i], nullptr, 0));
                        break;
                        
                    default:  // string
                        vr.emplace_back(std::string(rowFields[i]));
                        break;
                }
            }
            // Insert the row vector to the result vector
            v.emplace_back(vr);
        }
        
        CsvParser_destroy_row(row);
    }
    
    CsvParser_destroy(csvparser);
    return v;
}

chaiscript::ModulePtr SMNChai::APIUtils::smnchai_api_utils_math(chaiscript::ModulePtr m)
{
    using namespace chaiscript::extras::math;
    
    // TRIG FUNCTIONS
    cos<double, double>(m);
    cos<float, float>(m);
    cos<long double, long double>(m);
    
    sin<double, double>(m);
    sin<float, float>(m);
    sin<long double, long double>(m);
    
    tan<double, double>(m);
    tan<float, float>(m);
    tan<long double, long double>(m);
    
    acos<double, double>(m);
    acos<float, float>(m);
    acos<long double, long double>(m);
    
    asin<double, double>(m);
    asin<float, float>(m);
    asin<long double, long double>(m);
    
    atan<double, double>(m);
    atan<float, float>(m);
    atan<long double, long double>(m);
    
    atan2<double, double, double>(m);
    atan2<float, float, float>(m);
    atan2<long double, long double, long double>(m);
    
    // HYPERBOLIC FUNCTIONS
    cosh<double, double>(m);
    cosh<float, float>(m);
    cosh<long double, long double>(m);
    
    sinh<double, double>(m);
    sinh<float, float>(m);
    sinh<long double, long double>(m);
    
    tanh<double, double>(m);
    tanh<float, float>(m);
    tanh<long double, long double>(m);
    
    acosh<double, double>(m);
    acosh<float, float>(m);
    acosh<long double, long double>(m);
    
    asinh<double, double>(m);
    asinh<float, float>(m);
    asinh<long double, long double>(m);
    
    atanh<double, double>(m);
    atanh<float, float>(m);
    atanh<long double, long double>(m);
    
    // EXPONENTIAL AND LOGARITHMIC FUNCTIONS
    exp<double, double>(m);
    exp<float, float>(m);
    exp<long double, long double>(m);
    
    frexp<double, double, int *>(m);
    frexp<float, float, int *>(m);
    frexp<long double, long double, int *>(m);
    
    ldexp<double, double, int>(m);
    ldexp<float, float, int>(m);
    ldexp<long double, long double, int>(m);
    
    log<double, double>(m);
    log<float, float>(m);
    log<long double, long double>(m);
    
    log10<double, double>(m);
    log10<float, float>(m);
    log10<long double, long double>(m);
    
    modf<double, double, double *>(m);
    modf<float, float, float *>(m);
    modf<long double, long double, long double *>(m);
    
    exp2<double, double>(m);
    exp2<float, float>(m);
    exp2<long double, long double>(m);
    
    expm1<double, double>(m);
    expm1<float, float>(m);
    expm1<long double, long double>(m);
    
    ilogb<int, double>(m);
    ilogb<int, float>(m);
    ilogb<int, long double>(m);
    
    log1p<double, double>(m);
    log1p<float, float>(m);
    log1p<long double, long double>(m);
    
    log2<double, double>(m);
    log2<float, float>(m);
    log2<long double, long double>(m);
    
    logb<double, double>(m);
    logb<float, float>(m);
    logb<long double, long double>(m);
    
    scalbn<double, double, int>(m);
    scalbn<float, float, int>(m);
    scalbn<long double, long double, int>(m);
    
    scalbln<double, double, long int>(m);
    scalbln<float, float, long int>(m);
    scalbln<long double, long double, long int>(m);
    
    // POWER FUNCTIONS
    pow<double, double, double>(m);
    pow<float, float, float>(m);
    pow<long double, long double, long double>(m);
    
    sqrt<double, double>(m);
    sqrt<float, float>(m);
    sqrt<long double, long double>(m);
    
    cbrt<double, double>(m);
    cbrt<float, float>(m);
    cbrt<long double, long double>(m);
    
    hypot<double, double, double>(m);
    hypot<float, float, float>(m);
    hypot<long double, long double, long double>(m);
    
    // ERROR AND GAMMA FUNCTIONS
    erf<double, double>(m);
    erf<float, float>(m);
    erf<long double, long double>(m);
    
    erfc<double, double>(m);
    erfc<float, float>(m);
    erfc<long double, long double>(m);
    
    tgamma<double, double>(m);
    tgamma<float, float>(m);
    tgamma<long double, long double>(m);
    
    lgamma<double, double>(m);
    lgamma<float, float>(m);
    lgamma<long double, long double>(m);
    
    // ROUNDING AND REMAINDER FUNCTIONS
    ceil<double, double>(m);
    ceil<float, float>(m);
    ceil<long double, long double>(m);
    
    floor<double, double>(m);
    floor<float, float>(m);
    floor<long double, long double>(m);
    
    fmod<double, double, double>(m);
    fmod<float, float, float>(m);
    fmod<long double, long double, long double>(m);
    
    trunc<double, double>(m);
    trunc<float, float>(m);
    trunc<long double, long double>(m);
    
    round<double, double>(m);
    round<float, float>(m);
    round<long double, long double>(m);
    
    lround<long int, double>(m);
    lround<long int, float>(m);
    lround<long int, long double>(m);
    
    // long long ints do not work
    llround<long long int, double>(m);
    llround<long long int, float>(m);
    llround<long long int, long double>(m);
    
    rint<double, double>(m);
    rint<float, float>(m);
    rint<long double, long double>(m);
    
    lrint<long int, double>(m);
    lrint<long int, float>(m);
    lrint<long int, long double>(m);
    
    // long long ints do not work
    llrint<long long int, double>(m);
    llrint<long long int, float>(m);
    llrint<long long int, long double>(m);
    
    nearbyint<double, double>(m);
    nearbyint<float, float>(m);
    nearbyint<long double, long double>(m);
    
    remainder<double, double, double>(m);
    remainder<float, float, float>(m);
    remainder<long double, long double, long double>(m);
    
    remquo<double, double, double, int *>(m);
    remquo<float, float, float, int *>(m);
    remquo<long double, long double, long double, int *>(m);
    
    // FLOATING-POINT MANIPULATION FUNCTIONS
    copysign<double, double, double>(m);
    copysign<float, float, float>(m);
    copysign<long double, long double, long double>(m);
    
    nan<double, const char*>(m);
    
    nextafter<double, double, double>(m);
    nextafter<float, float, float>(m);
    nextafter<long double, long double, long double>(m);
    
    nexttoward<double, double, long double>(m);
    nexttoward<float, float, long double>(m);
    nexttoward<long double, long double, long double>(m);
    
    // MINIMUM, MAXIMUM, DIFFERENCE FUNCTIONS
    fdim<double, double, double>(m);
    fdim<float, float, float>(m);
    fdim<long double, long double, long double>(m);
    
    fmax<double, double, double>(m);
    fmax<float, float, float>(m);
    fmax<long double, long double, long double>(m);
    
    fmin<double, double, double>(m);
    fmin<float, float, float>(m);
    fmin<long double, long double, long double>(m);
    
    // OTHER FUNCTIONS
    fabs<double, double>(m);
    fabs<float, float>(m);
    fabs<long double, long double>(m);
    
    abs<double, double>(m);
    abs<float, float>(m);
    abs<long double, long double>(m);
    
    fma<double, double, double, double>(m);
    fma<float, float, float, float>(m);
    fma<long double, long double, long double, long double>(m);
    
    // CLASSIFICATION FUNCTIONS
    fpclassify<int, float>(m);
    fpclassify<int, double>(m);
    fpclassify<int, long double>(m);
    
    isfinite<bool, float>(m);
    isfinite<bool, double>(m);
    isfinite<bool, long double>(m);
    
    isinf<bool, float>(m);
    isinf<bool, double>(m);
    isinf<bool, long double>(m);
    
    isnan<bool, float>(m);
    isnan<bool, double>(m);
    isnan<bool, long double>(m);
    
    isnormal<bool, float>(m);
    isnormal<bool, double>(m);
    isnormal<bool, long double>(m);
    
    signbit<bool, float>(m);
    signbit<bool, double>(m);
    signbit<bool, long double>(m);
    
    // COMPARISON FUNCTIONS
    isgreater<bool, double, double>(m);
    isgreater<bool, float, float>(m);
    isgreater<bool, long double, long double>(m);
    
    isgreaterequal<bool, double, double>(m);
    isgreaterequal<bool, float, float>(m);
    isgreaterequal<bool, long double, long double>(m);
    
    isless<bool, double, double>(m);
    isless<bool, float, float>(m);
    isless<bool, long double, long double>(m);
    
    islessequal<bool, double, double>(m);
    islessequal<bool, float, float>(m);
    islessequal<bool, long double, long double>(m);
    
    islessgreater<bool, double, double>(m);
    islessgreater<bool, float, float>(m);
    islessgreater<bool, long double, long double>(m);
    
    isunordered<bool, double, double>(m);
    isunordered<bool, float, float>(m);
    isunordered<bool, long double, long double>(m);
    
    return m;
}

chaiscript::ModulePtr SMNChai::APIUtils::smnchai_api_utils_fixes(chaiscript::ModulePtr m)
{
    // Set the value at a given index of a vector in Chaiscript
    m->add(chaiscript::fun([](SMNChai::APIUtils::TChaiVector &v, std::size_t idx, const chaiscript::Boxed_Value &b){
        v[idx].assign(b);
    }), "set");
    
    return m;
}


chaiscript::ModulePtr SMNChai::APIUtils::smnchai_api_utils_misc(chaiscript::ModulePtr m)
{
    // Delay the execution by a number of milliseconds
    m->add(chaiscript::fun([](unsigned long t){ std::this_thread::sleep_for(std::chrono::milliseconds(t)); }), "delay");
    
    return m;
}
