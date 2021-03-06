/*
 *  mpi_manager.cpp
 *
 *  This file is part of NEST.
 *
 *  Copyright (C) 2004 The NEST Initiative
 *
 *  NEST is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  NEST is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NEST.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mpi_manager.h"

// C++ includes:
#include <limits>
#include <numeric>

// Includes from libnestutil:
#include "compose.hpp"
#include "logging.h"
#include "stopwatch.h"

// Includes from nestkernel:
#include "kernel_manager.h"
#include "mpi_manager_impl.h"
#include "nest_types.h"
#include "nodelist.h"

// Includes from sli:
#include "dictutils.h"

#ifdef HAVE_MPI

template <>
MPI_Datatype MPI_Type< nest::int_t >::type = MPI_INT;
template <>
MPI_Datatype MPI_Type< nest::double_t >::type = MPI_DOUBLE;
template <>
MPI_Datatype MPI_Type< nest::long_t >::type = MPI_LONG;
template <>
MPI_Datatype MPI_Type< nest::uint_t >::type = MPI_INT;
template <>
MPI_Datatype MPI_Type< nest::ulong_t >::type = MPI_UNSIGNED_LONG;

#endif /* #ifdef HAVE_MPI */

nest::MPIManager::MPIManager()
  : num_processes_( 1 )
  , rank_( 0 )
  , n_rec_procs_( 0 )
  , n_sim_procs_( 0 )
  , send_buffer_size_( 1 )
  , recv_buffer_size_( 1 )
  , use_mpi_( false )
#ifdef HAVE_MPI
  , comm_step_( std::vector< int >() )
  , COMM_OVERFLOW_ERROR( std::numeric_limits< unsigned int >::max() )
  , comm( 0 )
  , MPI_OFFGRID_SPIKE( 0 )
#endif
{
}

void
nest::MPIManager::init_mpi( int* argc, char** argv[] )
{
#ifdef HAVE_MPI
  int init;
  MPI_Initialized( &init );

  if ( init == 0 )
  {
#ifdef HAVE_MUSIC
    kernel().music_manager.init_music( argc, argv );
    // get a communicator from MUSIC
    comm = kernel().music_manager.communicator();
#else  /* #ifdef HAVE_MUSIC */
    int provided_thread_level;
    MPI_Init_thread( argc, argv, MPI_THREAD_FUNNELED, &provided_thread_level );
    comm = MPI_COMM_WORLD;
#endif /* #ifdef HAVE_MUSIC */
  }

  MPI_Comm_size( comm, &num_processes_ );
  MPI_Comm_rank( comm, &rank_ );

  recv_buffer_size_ = send_buffer_size_ * get_num_processes();

  // create off-grid-spike type for MPI communication
  // creating derived datatype
  OffGridSpike::assert_datatype_compatibility_();
  MPI_Datatype source_types[ 2 ];
  int blockcounts[ 2 ];
  MPI_Aint offsets[ 2 ];
  MPI_Aint start_address, address;
  OffGridSpike ogs( 0, 0.0 );

  // OffGridSpike.gid
  offsets[ 0 ] = 0;
  source_types[ 0 ] = MPI_DOUBLE;
  blockcounts[ 0 ] = 1;

  // OffGridSpike.offset
  MPI_Get_address( &( ogs.gid_ ), &start_address );
  MPI_Get_address( &( ogs.offset_ ), &address );
  offsets[ 1 ] = address - start_address;
  source_types[ 1 ] = MPI_DOUBLE;
  blockcounts[ 1 ] = 1;

  // generate and commit struct
  MPI_Type_create_struct(
    2, blockcounts, offsets, source_types, &MPI_OFFGRID_SPIKE );
  MPI_Type_commit( &MPI_OFFGRID_SPIKE );

  use_mpi_ = true;
#endif /* #ifdef HAVE_MPI */
}

void
nest::MPIManager::initialize()
{
  set_num_rec_processes( 0, true );
}

void
nest::MPIManager::finalize()
{
}

void
nest::MPIManager::set_status( const DictionaryDatum& )
{
}

void
nest::MPIManager::get_status( DictionaryDatum& d )
{
  def< long >( d, "num_processes", num_processes_ );
  def< long >( d, "send_buffer_size", send_buffer_size_ );
  def< long >( d, "receive_buffer_size", recv_buffer_size_ );
}

void
nest::MPIManager::set_num_rec_processes( int nrp, bool called_by_reset )
{
  if ( kernel().node_manager.size() > 1 and not called_by_reset )
    throw KernelException(
      "Global spike detection mode must be enabled before nodes are created." );
  if ( nrp >= num_processes_ )
    throw KernelException(
      "Number of processes used for recording must be smaller than total "
      "number of processes." );
  n_rec_procs_ = nrp;
  n_sim_procs_ = num_processes_ - n_rec_procs_;

  kernel().rng_manager.create_rngs_();

  if ( nrp > 0 )
  {
    std::string msg = String::compose(
      "Entering global spike detection mode with %1 recording MPI processes "
      "and %2 simulating MPI processes.",
      n_rec_procs_,
      n_sim_procs_ );
    LOG( M_INFO, "MPIManager::set_num_rec_processes", msg );
  }
}

/**
 * Finish off MPI routines
 */
void
nest::MPIManager::mpi_finalize( int exitcode )
{
#ifdef HAVE_MPI
  MPI_Type_free( &MPI_OFFGRID_SPIKE );

  int finalized;
  MPI_Finalized( &finalized );

  int initialized;
  MPI_Initialized( &initialized );

  if ( finalized == 0 && initialized == 1 )
  {
    if ( exitcode == 0 )
    {
      kernel().music_manager.music_finalize(); // calls MPI_Finalize()
    }
    else
    {
      LOG( M_INFO,
        "MPIManager::finalize()",
        "Calling MPI_Abort() due to errors in the script." );
      mpi_abort( exitcode );
    }
  }
#endif /* #ifdef HAVE_MPI */
}


#ifdef HAVE_MPI

void
nest::MPIManager::mpi_abort( int exitcode )
{
  MPI_Abort( MPI_COMM_WORLD, exitcode );
}


std::string
nest::MPIManager::get_processor_name()
{
  char name[ 1024 ];
  int len;
  MPI_Get_processor_name( name, &len );
  name[ len ] = '\0';
  return name;
}

void
nest::MPIManager::communicate( std::vector< uint_t >& send_buffer,
  std::vector< uint_t >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  if ( get_num_processes() == 1 ) // purely thread-based
  {
    displacements[ 0 ] = 0;
    if ( static_cast< uint_t >( recv_buffer_size_ ) < send_buffer.size() )
    {
      recv_buffer_size_ = send_buffer_size_ = send_buffer.size();
      recv_buffer.resize( recv_buffer_size_ );
    }
    recv_buffer.swap( send_buffer );
  }
  else
  {
    communicate_Allgather( send_buffer, recv_buffer, displacements );
  }
}

void
nest::MPIManager::communicate_Allgather( std::vector< uint_t >& send_buffer,
  std::vector< uint_t >& recv_buffer,
  std::vector< int >& displacements )
{
  std::vector< int > recv_counts( get_num_processes(), send_buffer_size_ );

  // attempt Allgather
  if ( send_buffer.size() == static_cast< uint_t >( send_buffer_size_ ) )
  {
    MPI_Allgather( &send_buffer[ 0 ],
      send_buffer_size_,
      MPI_UNSIGNED,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_UNSIGNED,
      comm );
  }
  else
  {
    // DEC cxx required 0U literal, HEP 2007-03-26
    std::vector< uint_t > overflow_buffer( send_buffer_size_, 0U );
    overflow_buffer[ 0 ] = COMM_OVERFLOW_ERROR;
    overflow_buffer[ 1 ] = send_buffer.size();
    MPI_Allgather( &overflow_buffer[ 0 ],
      send_buffer_size_,
      MPI_UNSIGNED,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_UNSIGNED,
      comm );
  }
  // check for overflow condition
  int disp = 0;
  uint_t max_recv_count = send_buffer_size_;
  bool overflow = false;
  for ( int pid = 0; pid < get_num_processes(); ++pid )
  {
    uint_t block_disp = pid * send_buffer_size_;
    displacements[ pid ] = disp;
    if ( recv_buffer[ block_disp ] == COMM_OVERFLOW_ERROR )
    {
      overflow = true;
      recv_counts[ pid ] = recv_buffer[ block_disp + 1 ];
      if ( static_cast< uint_t >( recv_counts[ pid ] ) > max_recv_count )
      {
        max_recv_count = recv_counts[ pid ];
      }
    }
    disp += recv_counts[ pid ];
  }

  // do Allgatherv if necessary
  if ( overflow )
  {
    recv_buffer.resize( disp, 0 );
    MPI_Allgatherv( &send_buffer[ 0 ],
      send_buffer.size(),
      MPI_UNSIGNED,
      &recv_buffer[ 0 ],
      &recv_counts[ 0 ],
      &displacements[ 0 ],
      MPI_UNSIGNED,
      comm );
    send_buffer_size_ = max_recv_count;
    recv_buffer_size_ = send_buffer_size_ * get_num_processes();
  }
}

template < typename T >
void
nest::MPIManager::communicate_Allgather( std::vector< T >& send_buffer,
  std::vector< T >& recv_buffer,
  std::vector< int >& displacements )
{
  std::vector< int > recv_counts( get_num_processes(), send_buffer_size_ );

  // attempt Allgather
  if ( send_buffer.size() == static_cast< uint_t >( send_buffer_size_ ) )
    MPI_Allgather( &send_buffer[ 0 ],
      send_buffer_size_,
      MPI_Type< T >::type,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_Type< T >::type,
      comm );
  else
  {
    // DEC cxx required 0U literal, HEP 2007-03-26
    std::vector< uint_t > overflow_buffer( send_buffer_size_, 0U );
    overflow_buffer[ 0 ] = COMM_OVERFLOW_ERROR;
    overflow_buffer[ 1 ] = send_buffer.size();
    MPI_Allgather( &overflow_buffer[ 0 ],
      send_buffer_size_,
      MPI_Type< T >::type,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_Type< T >::type,
      comm );
  }
  // check for overflow condition
  int disp = 0;
  uint_t max_recv_count = send_buffer_size_;
  bool overflow = false;
  for ( int pid = 0; pid < get_num_processes(); ++pid )
  {
    uint_t block_disp = pid * send_buffer_size_;
    displacements[ pid ] = disp;
    if ( recv_buffer[ block_disp ] == COMM_OVERFLOW_ERROR )
    {
      overflow = true;
      recv_counts[ pid ] = recv_buffer[ block_disp + 1 ];
      if ( static_cast< uint_t >( recv_counts[ pid ] ) > max_recv_count )
        max_recv_count = recv_counts[ pid ];
    }
    disp += recv_counts[ pid ];
  }

  // do Allgatherv if necessary
  if ( overflow )
  {
    recv_buffer.resize( disp, 0 );
    MPI_Allgatherv( &send_buffer[ 0 ],
      send_buffer.size(),
      MPI_Type< T >::type,
      &recv_buffer[ 0 ],
      &recv_counts[ 0 ],
      &displacements[ 0 ],
      MPI_Type< T >::type,
      comm );
    send_buffer_size_ = max_recv_count;
    recv_buffer_size_ = send_buffer_size_ * get_num_processes();
  }
}

void
nest::MPIManager::communicate( std::vector< OffGridSpike >& send_buffer,
  std::vector< OffGridSpike >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  if ( get_num_processes() == 1 ) // purely thread-based
  {
    displacements[ 0 ] = 0;
    if ( static_cast< uint_t >( recv_buffer_size_ ) < send_buffer.size() )
    {
      recv_buffer_size_ = send_buffer_size_ = send_buffer.size();
      recv_buffer.resize( recv_buffer_size_ );
    }
    recv_buffer.swap( send_buffer );
  }
  else
  {
    communicate_Allgather( send_buffer, recv_buffer, displacements );
  }
}

void
nest::MPIManager::communicate_Allgather(
  std::vector< OffGridSpike >& send_buffer,
  std::vector< OffGridSpike >& recv_buffer,
  std::vector< int >& displacements )
{
  std::vector< int > recv_counts( get_num_processes(), send_buffer_size_ );
  // attempt Allgather
  if ( send_buffer.size() == static_cast< uint_t >( send_buffer_size_ ) )
    MPI_Allgather( &send_buffer[ 0 ],
      send_buffer_size_,
      MPI_OFFGRID_SPIKE,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_OFFGRID_SPIKE,
      comm );
  else
  {
    std::vector< OffGridSpike > overflow_buffer( send_buffer_size_ );
    overflow_buffer[ 0 ] = OffGridSpike( COMM_OVERFLOW_ERROR, 0.0 );
    overflow_buffer[ 1 ] = OffGridSpike( send_buffer.size(), 0.0 );
    MPI_Allgather( &overflow_buffer[ 0 ],
      send_buffer_size_,
      MPI_OFFGRID_SPIKE,
      &recv_buffer[ 0 ],
      send_buffer_size_,
      MPI_OFFGRID_SPIKE,
      comm );
  }

  // check for overflow condition
  int disp = 0;
  uint_t max_recv_count = send_buffer_size_;
  bool overflow = false;
  for ( int pid = 0; pid < get_num_processes(); ++pid )
  {
    uint_t block_disp = pid * send_buffer_size_;
    displacements[ pid ] = disp;
    if ( ( recv_buffer[ block_disp ] ).get_gid() == COMM_OVERFLOW_ERROR )
    {
      overflow = true;
      recv_counts[ pid ] = ( recv_buffer[ block_disp + 1 ] ).get_gid();
      if ( static_cast< uint_t >( recv_counts[ pid ] ) > max_recv_count )
        max_recv_count = recv_counts[ pid ];
    }
    disp += recv_counts[ pid ];
  }

  // do Allgatherv if necessary
  if ( overflow )
  {
    recv_buffer.resize( disp );
    MPI_Allgatherv( &send_buffer[ 0 ],
      send_buffer.size(),
      MPI_OFFGRID_SPIKE,
      &recv_buffer[ 0 ],
      &recv_counts[ 0 ],
      &displacements[ 0 ],
      MPI_OFFGRID_SPIKE,
      comm );
    send_buffer_size_ = max_recv_count;
    recv_buffer_size_ = send_buffer_size_ * get_num_processes();
  }
}

void
nest::MPIManager::communicate( std::vector< double_t >& send_buffer,
  std::vector< double_t >& recv_buffer,
  std::vector< int >& displacements )
{
  // get size of buffers
  std::vector< int > n_nodes( get_num_processes() );
  n_nodes[ get_rank() ] = send_buffer.size();
  communicate( n_nodes );
  // Set up displacements vector.
  displacements.resize( get_num_processes(), 0 );
  for ( int i = 1; i < get_num_processes(); ++i )
    displacements.at( i ) = displacements.at( i - 1 ) + n_nodes.at( i - 1 );

  // Calculate total number of node data items to be gathered.
  size_t n_globals = std::accumulate( n_nodes.begin(), n_nodes.end(), 0 );

  if ( n_globals != 0 )
  {
    recv_buffer.resize( n_globals, 0.0 );
    communicate_Allgatherv( send_buffer, recv_buffer, displacements, n_nodes );
  }
  else
  {
    recv_buffer.clear();
  }
}

void
nest::MPIManager::communicate( std::vector< ulong_t >& send_buffer,
  std::vector< ulong_t >& recv_buffer,
  std::vector< int >& displacements )
{
  // get size of buffers
  std::vector< int > n_nodes( num_processes_ );
  n_nodes[ rank_ ] = send_buffer.size();
  communicate( n_nodes );
  // Set up displacements vector.
  displacements.resize( num_processes_, 0 );
  for ( int i = 1; i < num_processes_; ++i )
    displacements.at( i ) = displacements.at( i - 1 ) + n_nodes.at( i - 1 );

  // Calculate total number of node data items to be gathered.
  size_t n_globals = std::accumulate( n_nodes.begin(), n_nodes.end(), 0 );

  if ( n_globals != 0 )
  {
    recv_buffer.resize( n_globals, 0.0 );
    communicate_Allgatherv( send_buffer, recv_buffer, displacements, n_nodes );
  }
  else
  {
    recv_buffer.clear();
  }
}

void
nest::MPIManager::communicate( std::vector< int_t >& send_buffer,
  std::vector< int_t >& recv_buffer,
  std::vector< int >& displacements )
{
  // get size of buffers
  std::vector< int > n_nodes( num_processes_ );
  n_nodes[ rank_ ] = send_buffer.size();
  communicate( n_nodes );
  // Set up displacements vector.
  displacements.resize( num_processes_, 0 );
  for ( int i = 1; i < num_processes_; ++i )
    displacements.at( i ) = displacements.at( i - 1 ) + n_nodes.at( i - 1 );

  // Calculate total number of node data items to be gathered.
  size_t n_globals = std::accumulate( n_nodes.begin(), n_nodes.end(), 0 );

  if ( n_globals != 0 )
  {
    recv_buffer.resize( n_globals, 0.0 );
    communicate_Allgatherv( send_buffer, recv_buffer, displacements, n_nodes );
  }
  else
  {
    recv_buffer.clear();
  }
}

void
nest::MPIManager::communicate( double_t send_val,
  std::vector< double_t >& recv_buffer )
{
  recv_buffer.resize( get_num_processes() );
  MPI_Allgather(
    &send_val, 1, MPI_DOUBLE, &recv_buffer[ 0 ], 1, MPI_DOUBLE, comm );
}


/**
 * communicate function for sending set-up information
 */
void
nest::MPIManager::communicate( std::vector< int_t >& buffer )
{
  communicate_Allgather( buffer );
}

void
nest::MPIManager::communicate( std::vector< long_t >& buffer )
{
  communicate_Allgather( buffer );
}

void
nest::MPIManager::communicate_Allgather( std::vector< int_t >& buffer )
{
  // avoid aliasing, see http://www.mpi-forum.org/docs/mpi-11-html/node10.html
  int_t my_val = buffer[ get_rank() ];
  MPI_Allgather( &my_val, 1, MPI_INT, &buffer[ 0 ], 1, MPI_INT, comm );
}

/*
 * Sum across all rank
 */
void
nest::MPIManager::communicate_Allreduce_sum_in_place( double_t buffer )
{
  MPI_Allreduce(
    MPI_IN_PLACE, &buffer, 1, MPI_Type< double_t >::type, MPI_SUM, comm );
}

void
nest::MPIManager::communicate_Allreduce_sum_in_place(
  std::vector< double_t >& buffer )
{
  MPI_Allreduce( MPI_IN_PLACE,
    &buffer[ 0 ],
    buffer.size(),
    MPI_Type< double_t >::type,
    MPI_SUM,
    comm );
}

void
nest::MPIManager::communicate_Allreduce_sum_in_place(
  std::vector< int_t >& buffer )
{
  MPI_Allreduce( MPI_IN_PLACE,
    &buffer[ 0 ],
    buffer.size(),
    MPI_Type< int_t >::type,
    MPI_SUM,
    comm );
}

void
nest::MPIManager::communicate_Allreduce_sum(
  std::vector< double_t >& send_buffer,
  std::vector< double_t >& recv_buffer )
{
  assert( recv_buffer.size() == send_buffer.size() );
  MPI_Allreduce( &send_buffer[ 0 ],
    &recv_buffer[ 0 ],
    send_buffer.size(),
    MPI_Type< double_t >::type,
    MPI_SUM,
    comm );
}

void
nest::MPIManager::communicate_Allgather( std::vector< long_t >& buffer )
{
  // avoid aliasing, see http://www.mpi-forum.org/docs/mpi-11-html/node10.html
  long_t my_val = buffer[ get_rank() ];
  MPI_Allgather( &my_val, 1, MPI_LONG, &buffer[ 0 ], 1, MPI_LONG, comm );
}

/**
 * Ensure all processes have reached the same stage by waiting until all
 * processes have sent a dummy message to process 0.
 */
void
nest::MPIManager::synchronize()
{
  MPI_Barrier( comm );
}

void
nest::MPIManager::test_link( int sender, int receiver )
{
  assert( sender < get_num_processes() && receiver < get_num_processes() );

  if ( get_num_processes() > 1 )
  {
    long dummy = 1;
    MPI_Status status;

    if ( get_rank() == sender )
      MPI_Ssend( &dummy, 1, MPI_LONG, receiver, 0, comm );
    else if ( get_rank() == receiver )
    {
      MPI_Recv( &dummy, 1, MPI_LONG, sender, 0, comm, &status );
      // std::cerr << "link between " << sender << " and " << receiver << "
      // works" << std::endl;
    }
  }
}

void
nest::MPIManager::test_links()
{
  for ( int i = 0; i < get_num_processes(); ++i )
    for ( int j = 0; j < get_num_processes(); ++j )
      if ( i != j )
        test_link( i, j );
  // std::cerr << "all links are working" << std::endl;
}

// grng_synchrony: called at the beginning of each simulate
bool
nest::MPIManager::grng_synchrony( unsigned long process_rnd_number )
{
  if ( get_num_processes() > 1 )
  {
    std::vector< unsigned long > rnd_numbers( get_num_processes() );
    MPI_Allgather( &process_rnd_number,
      1,
      MPI_UNSIGNED_LONG,
      &rnd_numbers[ 0 ],
      1,
      MPI_UNSIGNED_LONG,
      comm );
    // compare all rnd numbers
    for ( uint_t i = 1; i < rnd_numbers.size(); ++i )
    {
      if ( rnd_numbers[ i - 1 ] != rnd_numbers[ i ] )
      {
        return false;
      }
    }
  }
  return true;
}

// average communication time for a packet size of num_bytes using Allgather
nest::double_t
nest::MPIManager::time_communicate( int num_bytes, int samples )
{
  if ( get_num_processes() == 1 )
    return 0.0;
  uint_t packet_length = num_bytes / sizeof( uint_t );
  if ( packet_length < 1 )
    packet_length = 1;
  std::vector< uint_t > test_send_buffer( packet_length );
  std::vector< uint_t > test_recv_buffer( packet_length * get_num_processes() );
  // start time measurement here
  Stopwatch foo;
  foo.start();
  for ( int i = 0; i < samples; ++i )
    MPI_Allgather( &test_send_buffer[ 0 ],
      packet_length,
      MPI_UNSIGNED,
      &test_recv_buffer[ 0 ],
      packet_length,
      MPI_UNSIGNED,
      MPI_COMM_WORLD );
  // finish time measurement here
  foo.stop();
  return foo.elapsed() / samples;
}

// average communication time for a packet size of num_bytes using Allgatherv
nest::double_t
nest::MPIManager::time_communicatev( int num_bytes, int samples )
{
  if ( get_num_processes() == 1 )
    return 0.0;
  uint_t packet_length = num_bytes / sizeof( uint_t );
  if ( packet_length < 1 )
    packet_length = 1;
  std::vector< uint_t > test_send_buffer( packet_length );
  std::vector< uint_t > test_recv_buffer( packet_length * get_num_processes() );
  std::vector< int > n_nodes( get_num_processes(), packet_length );
  std::vector< int > displacements( get_num_processes(), 0 );

  for ( int i = 1; i < get_num_processes(); ++i )
    displacements.at( i ) = displacements.at( i - 1 ) + n_nodes.at( i - 1 );

  // start time measurement here
  Stopwatch foo;
  foo.start();
  for ( int i = 0; i < samples; ++i )
    communicate_Allgatherv(
      test_send_buffer, test_recv_buffer, displacements, n_nodes );

  // finish time measurement here
  foo.stop();
  return foo.elapsed() / samples;
}

// average communication time for a packet size of num_bytes
nest::double_t
nest::MPIManager::time_communicate_offgrid( int num_bytes, int samples )
{
  if ( get_num_processes() == 1 )
    return 0.0;
  uint_t packet_length = num_bytes / sizeof( OffGridSpike );
  if ( packet_length < 1 )
    packet_length = 1;
  std::vector< OffGridSpike > test_send_buffer( packet_length );
  std::vector< OffGridSpike > test_recv_buffer(
    packet_length * get_num_processes() );
  // start time measurement here
  Stopwatch foo;
  foo.start();
  for ( int i = 0; i < samples; ++i )
    MPI_Allgather( &test_send_buffer[ 0 ],
      packet_length,
      MPI_OFFGRID_SPIKE,
      &test_recv_buffer[ 0 ],
      packet_length,
      MPI_OFFGRID_SPIKE,
      MPI_COMM_WORLD );
  // finish time measurement here
  foo.stop();
  return foo.elapsed() / samples;
}

// average communication time for a packet size of num_bytes using Alltoall
nest::double_t
nest::MPIManager::time_communicate_alltoall( int num_bytes, int samples )
{
  if ( get_num_processes() == 1 )
    return 0.0;
  uint_t packet_length =
    num_bytes / sizeof( uint_t ); // this size should be sent to each process
  uint_t total_packet_length = packet_length
    * get_num_processes(); // total size of send and receive buffers
  if ( total_packet_length < 1 )
    total_packet_length = 1;
  std::vector< uint_t > test_send_buffer( total_packet_length );
  std::vector< uint_t > test_recv_buffer( total_packet_length );
  // start time measurement here
  Stopwatch foo;
  foo.start();
  for ( int i = 0; i < samples; ++i )
  {
    MPI_Alltoall( &test_send_buffer[ 0 ],
      packet_length,
      MPI_UNSIGNED,
      &test_recv_buffer[ 0 ],
      packet_length,
      MPI_UNSIGNED,
      MPI_COMM_WORLD );
  }
  // finish time measurement here
  foo.stop();
  return foo.elapsed() / samples;
}

// average communication time for a packet size of num_bytes using Alltoallv
nest::double_t
nest::MPIManager::time_communicate_alltoallv( int num_bytes, int samples )
{
  if ( get_num_processes() == 1 )
    return 0.0;
  uint_t packet_length =
    num_bytes / sizeof( uint_t ); // this size should be sent to each process
  uint_t total_packet_length = packet_length
    * get_num_processes(); // total size of send and receive buffers
  if ( total_packet_length < 1 )
    total_packet_length = 1;
  std::vector< uint_t > test_send_buffer( total_packet_length );
  std::vector< uint_t > test_recv_buffer( total_packet_length );
  std::vector< int > n_nodes( get_num_processes(), packet_length );
  std::vector< int > displacements( get_num_processes(), 0 );

  for ( int i = 1; i < get_num_processes(); ++i )
    displacements.at( i ) = displacements.at( i - 1 ) + n_nodes.at( i - 1 );

  // start time measurement here
  Stopwatch foo;
  foo.start();
  for ( int i = 0; i < samples; ++i )
  {
    MPI_Alltoallv( &test_send_buffer[ 0 ],
      &n_nodes[ 0 ],
      &displacements[ 0 ],
      MPI_UNSIGNED,
      &test_recv_buffer[ 0 ],
      &n_nodes[ 0 ],
      &displacements[ 0 ],
      MPI_UNSIGNED,
      MPI_COMM_WORLD );
  }
  // finish time measurement here
  foo.stop();
  return foo.elapsed() / samples;
}


void
nest::MPIManager::communicate_connector_properties( DictionaryDatum& dict )
{
  // Confirm that we're having a MPI process
  if ( get_num_processes() > 1 )
  {
    // Move local dictionary values to temporary storage vectors.
    std::vector< nest::long_t > targets =
      getValue< std::vector< nest::long_t > >( dict, "targets" );

    std::vector< nest::double_t > weights =
      getValue< std::vector< nest::double_t > >( dict, "weights" );

    std::vector< nest::double_t > delays =
      getValue< std::vector< nest::double_t > >( dict, "delays" );

    std::vector< nest::long_t > receptors =
      getValue< std::vector< nest::long_t > >( dict, "receptors" );

    // Calculate size of communication buffers (number of connections).
    std::vector< nest::int_t > num_connections( get_num_processes() );
    num_connections[ get_rank() ] = targets.size();
    communicate( num_connections );

    // Set up displacements vector.
    std::vector< int > displacements( get_num_processes(), 0 );

    for ( size_t i = 1; i < num_connections.size(); ++i )
      displacements.at( i ) =
        displacements.at( i - 1 ) + num_connections.at( i - 1 );

    // Calculate sum of global connections.
    nest::int_t num_connections_sum =
      std::accumulate( num_connections.begin(), num_connections.end(), 0 );

    if ( num_connections_sum != 0 )
    {
      // Create global buffers.
      std::vector< nest::long_t > targets_result( num_connections_sum, 0 );

      std::vector< nest::long_t > receptors_result( num_connections_sum, 0 );

      std::vector< nest::double_t > weights_result( num_connections_sum, 0 );

      std::vector< nest::double_t > delays_result( num_connections_sum, 0 );

      // Start communication.
      communicate_Allgatherv< nest::long_t >(
        targets, targets_result, displacements, num_connections );

      communicate_Allgatherv< nest::long_t >(
        receptors, receptors_result, displacements, num_connections );

      communicate_Allgatherv< nest::double_t >(
        weights, weights_result, displacements, num_connections );

      communicate_Allgatherv< nest::double_t >(
        delays, delays_result, displacements, num_connections );

      // Save global values in input dictionary.
      ( *dict )[ "targets" ] = targets_result;
      ( *dict )[ "receptors" ] = receptors_result;
      ( *dict )[ "weights" ] = weights_result;
      ( *dict )[ "delays" ] = delays_result;
    }
  }
}

#else /* #ifdef HAVE_MPI */

/**
 * communicate (on-grid) if compiled without MPI
 */
void
nest::MPIManager::communicate( std::vector< uint_t >& send_buffer,
  std::vector< uint_t >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  displacements[ 0 ] = 0;
  if ( static_cast< size_t >( recv_buffer_size_ ) < send_buffer.size() )
  {
    recv_buffer_size_ = send_buffer_size_ = send_buffer.size();
    recv_buffer.resize( recv_buffer_size_ );
  }
  recv_buffer.swap( send_buffer );
}

/**
 * communicate (off-grid) if compiled without MPI
 */
void
nest::MPIManager::communicate( std::vector< OffGridSpike >& send_buffer,
  std::vector< OffGridSpike >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  displacements[ 0 ] = 0;
  if ( static_cast< size_t >( recv_buffer_size_ ) < send_buffer.size() )
  {
    recv_buffer_size_ = send_buffer_size_ = send_buffer.size();
    recv_buffer.resize( recv_buffer_size_ );
  }
  recv_buffer.swap( send_buffer );
}

void
nest::MPIManager::communicate( std::vector< double_t >& send_buffer,
  std::vector< double_t >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  displacements[ 0 ] = 0;
  recv_buffer.swap( send_buffer );
}

void
nest::MPIManager::communicate( std::vector< ulong_t >& send_buffer,
  std::vector< ulong_t >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  displacements[ 0 ] = 0;
  recv_buffer.swap( send_buffer );
}

void
nest::MPIManager::communicate( std::vector< int_t >& send_buffer,
  std::vector< int_t >& recv_buffer,
  std::vector< int >& displacements )
{
  displacements.resize( num_processes_, 0 );
  displacements[ 0 ] = 0;
  recv_buffer.swap( send_buffer );
}

void
nest::MPIManager::communicate( double_t send_val,
  std::vector< double_t >& recv_buffer )
{
  recv_buffer.resize( 1 );
  recv_buffer[ 0 ] = send_val;
}

void
nest::MPIManager::communicate_Allreduce_sum_in_place( double_t buffer )
{
}

void
nest::MPIManager::communicate_Allreduce_sum_in_place(
  std::vector< double_t >& buffer )
{
}

void
nest::MPIManager::communicate_Allreduce_sum_in_place(
  std::vector< int_t >& buffer )
{
}

void
nest::MPIManager::communicate_Allreduce_sum(
  std::vector< double_t >& send_buffer,
  std::vector< double_t >& recv_buffer )
{
  recv_buffer.swap( send_buffer );
}

#endif /* #ifdef HAVE_MPI */
