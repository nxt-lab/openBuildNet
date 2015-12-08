/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief IO utility functions for Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef __nodechai__chaiscript_io__
#define __nodechai__chaiscript_io__

#include <cstdio>
#include <vector>

namespace chaiscript {
    class Boxed_Value;
    class Module;
}

namespace NodeChai {
    namespace extras {
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
        
        /** \brief Class to write numeric data to CSV file.
         */
        class CSVFileWriter {
            std::string m_delimiter{","};   ///< The delimiter, can be a string [default: comma]
            char m_newline{'\n'};   ///< New line character
            
            std::string m_num_formatter{"%g"};  ///< Format string for (real-valued) numbers, cf. fprintf [default: %g]
            
            bool m_linebegin{true};   ///< True if a new line has just been created -> should not insert a delimiter before writing a new element
            
            FILE * m_file = NULL;  ///< The FILE object
            
            /** Check if a file is open and valid; otherwise throw an exception.
             This method is used internally by any method that needs to operate on the file.
             */
            void check_file_status() {
                if (m_file == NULL || ferror(m_file)) {
                    throw nodechai_exception("CSVFileWriter: file is not open or has error.");
                }
            }
            
            /** Insert a delimiter if this is not at the beginning of a line/row. Used internally.
             File must already be opened and valid. */
            void insert_delimiter() {
                if (m_linebegin) {
                    m_linebegin = false;
                } else {
                    // Not at the beginning of a new line/row -> add delimiter first
                    fputs(m_delimiter.c_str(), m_file);
                }
            }
        public:
            /** Construct the CSV file writer with given file name.
             \param t_filename File name
             \param t_append True if the file is to be appended (default: false).
             */
            CSVFileWriter(const std::string& t_filename, bool t_append = false);
            
            /** Make sure that the file is closed upon destruction. */
            virtual ~CSVFileWriter();
            
            /** Close the file manually. */
            void close();
            
            /** Set the formatter string for (real) numbers, cf. fprintf. */
            void setf(const std::string& tf) {
                m_num_formatter = tf;
            }
            
            /** Set the delimiter character. */
            void set_delim(const std::string& s) {
                m_delimiter = s;
            }
            
            /** Insert a new line / row in the CSV file. */
            void new_line();
            
            /** Insert a scalar number to the CSV file, on the current line. */
            void add_double(double d);
            void add_integer(int i);
            
            /** Insert a matrix/vector to the CSV file.
             If the parameter is a vector, either row or column, its elements are written to the current line/row without any new line added.
             If the parameter is not a vector, its elements are written to the CSV file, row by row, starting from the current line; new lines are created accordingly.
             */
            void add_matrix(const Eigen::MatrixXd& m);
            
        };
        
        std::shared_ptr<chaiscript::Module> bind_CSVFileWriter(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>());
    }
}


#endif /* defined(__nodechai__chaiscript_io__) */
