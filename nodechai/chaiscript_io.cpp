/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief IO utility functions for Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <csvparser/csvparser.h>
#include <Eigen/Dense>

#include "nodechai.h"
#include "chaiscript_io.h"
#include <chaiscript/chaiscript.hpp>

using namespace NodeChai;


//**************************************
//* Implementation of loading a CSV file
//**************************************
NodeChai::extras::TChaiVector NodeChai::extras::load_csv_into_chai(const std::string& t_file,
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
        throw nodechai_exception("The format string must be specified.");
    }
    
    std::size_t pos = t_format.find_first_not_of("dis");
    if (pos != std::string::npos) {
        throw nodechai_exception(std::string("The format string contains an invalid character: ") + t_format[pos]);
    }
    
    auto nfields = t_format.size();
    
    CsvParser *csvparser = CsvParser_new(t_file.c_str(), delimiter, t_header);
    
    if (t_header) {
        CsvRow *header = CsvParser_getHeader(csvparser);
        if (header == NULL) {
            throw nodechai_exception(CsvParser_getErrorMessage(csvparser));
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
    NodeChai::extras::TChaiVector v;  // The entire table
    NodeChai::extras::TChaiVector vr; // Each row
    
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


//*****************************************
//* Implementation of writing to CSV file
//*****************************************

NodeChai::extras::CSVFileWriter::CSVFileWriter(const std::string& t_filename, bool t_append) {
    if (t_filename.empty()) {
        throw nodechai_exception("CSVFileWriter: a valid file name must be given.");
    }
    // Attempt to open the file
    m_file = fopen(t_filename.c_str(), t_append?"a":"w");
    if (m_file == NULL) {
        // Error
        throw nodechai_exception("CSVFileWriter: could not open file '" + t_filename + "' for writing.");
    }
    // At this point, the file can be written to
}

NodeChai::extras::CSVFileWriter::~CSVFileWriter() {
    // Close the file
    close();
}

void NodeChai::extras::CSVFileWriter::close() {
    if (m_file != NULL) {
        fclose(m_file);
        m_file = NULL;
    }
}

void NodeChai::extras::CSVFileWriter::new_line() {
    check_file_status();
    fputc(m_newline, m_file);
    m_linebegin = true;
}

void NodeChai::extras::CSVFileWriter::add_double(double d) {
    check_file_status();
    insert_delimiter();
    fprintf(m_file, m_num_formatter.c_str(), d);
}

void NodeChai::extras::CSVFileWriter::add_integer(int i) {
    check_file_status();
    insert_delimiter();
    fprintf(m_file, m_num_formatter.c_str(), i);
}

void NodeChai::extras::CSVFileWriter::add_matrix(const Eigen::MatrixXd& m) {
    check_file_status();
    
    // Get the size of the matrix
    auto nrows = m.rows(), ncols = m.cols();
    if (nrows * ncols == 0) return;     // Empty -> don't write anything
    
    const char* num_formatter = m_num_formatter.c_str();
    
    if (nrows == 1 || ncols == 1) {
        // This is a vector, write it to the current line
        for (auto i = 0; i < m.size(); ++i) {
            insert_delimiter();
            fprintf(m_file, num_formatter, m(i));
        }
    } else {
        // This is a 2D matrix, write it as-is
        for (auto ri = 0; ri < nrows; ++ri) {
            if (ri > 0) {
                // Start a new row, but only from the second row
                fputc(m_newline, m_file);
                m_linebegin = true;
            }
            for (auto ci = 0; ci < ncols; ++ci) {
                insert_delimiter();
                fprintf(m_file, num_formatter, m(ri, ci));
            }
        }
    }
}

std::shared_ptr<chaiscript::Module> NodeChai::extras::bind_CSVFileWriter(std::shared_ptr<chaiscript::Module> m) {
    using namespace chaiscript;
    using namespace NodeChai::extras;
    
    m->add(user_type<CSVFileWriter>(), "CSVWriter");
    m->add(constructor<CSVFileWriter (const std::string&)>(), "CSVWriter");
    m->add(constructor<CSVFileWriter (const std::string&, bool)>(), "CSVWriter");
    
    m->add(fun(&CSVFileWriter::close), "close");
    
    m->add(fun(&CSVFileWriter::setf), "number_format");
    
    m->add(fun(&CSVFileWriter::set_delim), "set_delimiter");
    
    m->add(fun(&CSVFileWriter::new_line), "new_row");
    
    m->add(fun(&CSVFileWriter::add_double), "write");
    m->add(fun(&CSVFileWriter::add_integer), "write");
    m->add(fun(&CSVFileWriter::add_matrix), "write");
    
    return m;
}