#ifndef MINPQ_H
#define MINPQ_H

#include <iostream>
using namespace std;

#define MINPQ_INIT_NALLOCD 512
static inline int parent( int i ) { return ( i - 1 ) / 2; }
static inline int right( int i ) { return 2 * i + 2; }
static inline int left( int i ) { return 2 * i + 1; }


struct pq_node
{
	void* data;
	int key;
};

struct min_pq
{
	struct pq_node* pq_array;    /* array containing priority queue */
	int nallocd;                 /* number of elements allocated */
	int n;                       /**< number of elements in pq */
};

struct min_pq* minpq_init()
{
	struct min_pq* min_pq;

	min_pq = ( struct min_pq* )malloc( sizeof( struct min_pq ) );
	min_pq->pq_array = ( struct pq_node* )calloc( MINPQ_INIT_NALLOCD, sizeof( struct pq_node ) );
	min_pq->nallocd = MINPQ_INIT_NALLOCD;
	min_pq->n = 0;

	return min_pq;
}

void decrease_pq_node_key( struct pq_node* pq_array, int i, int key )
{
	struct pq_node tmp;

	if( key > pq_array[i].key )
		return;

	pq_array[i].key = key;
	while( i > 0  &&  pq_array[i].key < pq_array[parent(i)].key )
	{
		tmp = pq_array[parent(i)];
		pq_array[parent(i)] = pq_array[i];
		pq_array[i] = tmp;
		i = parent(i);
	}
}

void restore_minpq_order( struct pq_node* pq_array, int i, int n )
{
	struct pq_node tmp;
	int l, r, min = i;

	l = left( i );
	r = right( i );
	if( l < n )
		if( pq_array[l].key < pq_array[i].key )
			min = l;
	if( r < n )
		if( pq_array[r].key < pq_array[min].key )
			min = r;

	if( min != i )
	{
		tmp = pq_array[min];
		pq_array[min] = pq_array[i];
		pq_array[i] = tmp;
		restore_minpq_order( pq_array, min, n );
	}
}

int array_double( pq_node** array, int n, int size )
{
	pq_node* tmp;

	tmp = ( pq_node* )realloc( *array, 2 * n * size );
	if( ! tmp )
	{
		fprintf( stderr, "Warning: unable to allocate memory in array_double(),"
				" %s line %d\n", __FILE__, __LINE__ );
		if( *array )
			free( *array );
		*array = NULL;
		return 0;
	}
	*array = tmp;
	return n*2;
}


int minpq_insert( struct min_pq* min_pq, void* data, int key )
{
	int n = min_pq->n;

	if( min_pq->nallocd == n )
	{
		min_pq->nallocd = array_double( &min_pq->pq_array, min_pq->nallocd, sizeof( struct pq_node ) );
		if( ! min_pq->nallocd )
		{
			cout <<  "unable to allocate memory, %s, line %d" << endl;
			return 1;
		}
	}

	min_pq->pq_array[n].data = data;
	min_pq->pq_array[n].key = INT_MAX;
	decrease_pq_node_key( min_pq->pq_array, min_pq->n, key );
	min_pq->n++;

	return 0;
}

void* minpq_get_min( struct min_pq* min_pq )
{
	if( min_pq->n < 1 )
	{
		cout << "PQ empty" << endl;
		return NULL;
	}
	return min_pq->pq_array[0].data;
}

void* minpq_extract_min( struct min_pq* min_pq )
{
	void* data;

	if( min_pq->n < 1 )
	{
		cout << "PQ empty" << endl;
		return NULL;
	}
	data = min_pq->pq_array[0].data;
	min_pq->n--;
	min_pq->pq_array[0] = min_pq->pq_array[min_pq->n];
	restore_minpq_order( min_pq->pq_array, 0, min_pq->n );

	return data;
}

void minpq_release( struct min_pq** min_pq )
{
	if( ! min_pq )
	{
		cout << "NULL pointer" << endl;
		return;
	}
	if( *min_pq  &&  (*min_pq)->pq_array )
	{
		free( (*min_pq)->pq_array );
		free( *min_pq );
		*min_pq = NULL;
	}
}

#endif
