// This file is copyright Neil Hunt 2008
//
// You are free to use this code as you see fit,
// but I bear no liability for the use of this code.
//
// See http://blog.instedit.com/2008/05/openmp-utility-code.html
// for more details.

#ifndef SPLIT_RANGE_HPP
#define SPLIT_RANGE_HPP

#include <omp.h>
#include <boost/range.hpp>

namespace parallelism
{

///////////////////////////////////////////////
//
// The returned sub range is such that if this function is called 
// for each partition [0,partition_count), the entire "range"
// will be covered by all returned sub ranges, and distributed 
// amongst the partitions in the most even size distribution possible.
//
// The size parameter must specify the size of the range.
// This overload, accepting a size, is preferable where 
// range.size() may be expensive.
//
template<typename Range>
inline boost::iterator_range< typename Range::iterator > split_range( const Range& range, 
                                                     int partition_count,
                                                     int partition,
                                                     int size )
{
    typename Range::iterator begin = boost::begin( range );
    typename Range::iterator end = boost::end( range );

    if( partition_count > 1 )
    {
        int remainder = size % partition_count;
        int quotient = size / partition_count;

        if( partition < remainder )
        {
            std::advance( begin, partition * ( 1 + quotient ) );
            end = begin;
            std::advance( end, quotient + 1);
        }
        else
        {
            std::advance( begin, remainder + partition * quotient );
            end = begin;
            std::advance( end, quotient );
        }
    }

    return boost::make_iterator_range( begin, end );
}

///////////////////////////////////////////////
//
// The returned sub range is such that if this function is called 
// for each partition [0,partition_count), the entire "range"
// will be covered by all returned sub ranges, and distributed 
// amongst the partitions in the most even size distribution possible.
//
// Use this overload where range.size() is not expensive
// (i.e. Range::iterator models random_access_iterator )
//
template<typename Range>
inline boost::iterator_range< typename Range::iterator > split_range( const Range& range, 
                                                     int partition_count,
                                                     int partition )
{
    return split_range( range, partition_count, partition, range.size() );
}

///////////////////////////////////////////////
//
// This function should be called within a #pragma omp parallel
// block, and returns a sub_range of the input range.
//
// The returned sub range is such that if this function is called 
// by each thread in the parallel thread group, the entirety of "range"
// will be covered by all threads, and distributed amongst the threads
// in the most even size distribution possible.
//
// The size parameter must specify the size of the range.
// This overload, accepting a size, is preferable where 
// range.size() may be expensive.
//
template<typename Range >
inline boost::iterator_range< typename Range::iterator > split_range_openmp( const Range& range, int size )
{
    int thread_count = omp_get_num_threads();
    int thread = omp_get_thread_num();

    return split_range( range, thread_count, thread, size );
}

///////////////////////////////////////////////
//
// This function should be called within a #pragma omp parallel
// block, and returns a sub_range of the input range.
//
// The returned sub range is such that if this function is called 
// by each thread in the parallel thread group, the entirety of "range"
// will be covered by all threads, and distributed amongst the threads
// in the most even size() distribution possible.
//
// Use this overload where range.size() is not expensive
// (i.e. Range::iterator models random_access_iterator )
//
template<typename Range >
inline boost::iterator_range< typename Range::iterator > split_range_openmp( const Range& range )
{
    return split_range_openmp( range, range.size() );
}

} //namespace

#endif //SPLIT_RANGE_HPP