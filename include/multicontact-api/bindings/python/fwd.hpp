#ifndef __multicontact_api_python_fwd_hpp__
#define __multicontact_api_python_fwd_hpp__


// Silence a warning about a deprecated use of boost bind by boost python
// at least fo boost 1.73 to 1.75
// ref. https://github.com/stack-of-tasks/tsid/issues/128
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <boost/python.hpp>

 #undef BOOST_BIND_GLOBAL_PLACEHOLDERS

#endif  // ifndef __multicontact_api_python_utils_printable_hpp__
