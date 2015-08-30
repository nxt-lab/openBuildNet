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
