/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Utility Chaiscrip API for SMNChai.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef smnchai_smnchai_utils_h
#define smnchai_smnchai_utils_h

#include <chaiscript/chaiscript.hpp>
#include <chaiscript/dispatchkit/bootstrap.hpp>


namespace SMNChai {
    namespace APIUtils {
        
        typedef std::vector<chaiscript::Boxed_Value> TChaiVector;   ///< The C++ type of a Chaiscript's vector object
        
        /** \brief Load a CSV file into a Chaiscript Vector object.
         
         \param t_file Name of the CSV file.
         \param t_format A string that specifies how the fields should be parsed, one character for one field: 'd' for double, 'i' for int, 's' for string. Up to the length of t_format number of fields will be read: if a row has fewer than that, the rest in t_format will not be read; if a row has more, the rest of the row will be skipped.
         \param t_delimiter The delimiter characters (e.g. ",")
         \param t_header true if there is a header line at the beginning.
         \return The Vector object loaded from the CSV file; if there is any error, an exception will be thrown.
         */
        TChaiVector load_csv_into_chai(const std::string& t_file, const std::string& t_format,
                                       const std::string& t_delimiter, bool t_header);
        
        
        /** Create a Chaiscript module of utility API for IO. */
        inline chaiscript::ModulePtr smnchai_api_utils_io() {
            chaiscript::ModulePtr m(new chaiscript::Module());
            
            m->add(chaiscript::fun(&load_csv_into_chai), "load_csv_into_chai");
            
            return m;
        }
    }
}


#endif
