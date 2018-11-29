/*!*************************************************************************************************
\file    ObjectAllocator.cpp
\author  Seth Glaser
\date    1/11/2017
\par     Class: CS280-A
\par     Email: seth.g\@digipen.edu
\brief   This file holds the implementation for the ObjectAllocator.
***************************************************************************************************/

#include "ObjectAllocator.h"  // OAException, OAConfig, OAStats, ObjectAllocator
#include "cstring"            // strcpy


MemBlockInfo::MemBlockInfo(bool inUse, const char* userLabel, unsigned allocNumber) : in_use(inUse),
                           label(nullptrptr), alloc_num(allocNumber)
{
  // Points at the allocated memory
  char *newLabel = nullptrptr;
  
  // If there is a label from the user
  if(userLabel != nullptrptr)
  {
    try
    {
      // Tries to allocate the memory
      newLabel = new char[strlen(userLabel) + 1];
    }
    catch (std::bad_alloc&)
    {
      throw OAException(OAException::E_NO_MEMORY, "Out of memory for external header!");
    }
    
    // Copies the user's label to the allocated memory and points at it
    label = strcpy(newLabel, userLabel);
  }

}

MemBlockInfo::~MemBlockInfo()
{
  // If there was a label allocated
  if(label != nullptrptr)
    delete[] label;

  label = nullptrptr;
}



// Creates the ObjectManager per the specified values
// Throws an exception if the construction fails. (Memory allocation problem)
ObjectAllocator::ObjectAllocator(size_t ObjectSize, const OAConfig& config) : PageList_(nullptr),
                                 FreeList_(nullptr), Configuration_(config)
{
  // Set the values of the Stats
  Statistics_.ObjectSize_ = ObjectSize;
  Statistics_.PageSize_ = get_size_of_header() + (get_size_of_object() * config.ObjectsPerPage_);

  // Allocate the first page
  allocate_new_page();

}

// Destroys the ObjectManager (never throws)
ObjectAllocator::~ObjectAllocator()
{
  // While there are pages left
	while(PageList_ != nullptr)
	{
		GenericObject* temp = PageList_;
		PageList_ = PageList_->Next;

		delete [] temp;
	}
}

// Take an object from the free list and give it to the client (simulates new)
// Throws an exception if the object can't be allocated. (Memory allocation problem)
void *ObjectAllocator::Allocate(const char *label)
{
  // If they are using the CPPManager
  if(Configuration_.UseCPPMemManager_)
  {
    // Adjust stats
    Statistics_.Allocations_++;
    Statistics_.ObjectsInUse_++;
    Statistics_.FreeObjects_--;
    if (Statistics_.ObjectsInUse_ > Statistics_.MostObjects_)
      Statistics_.MostObjects_ = Statistics_.ObjectsInUse_;
    
    return operator new(Statistics_.ObjectSize_);
  }

  // If the free list isn't nullptr
  if(FreeList_ == nullptr)
  {
    // Try to make a new page
    allocate_new_page();
  }

  // Get the node to return
  void* returnNode = FreeList_;

  // Adjust the FreeList to look at the element after the returning node
  FreeList_ = FreeList_->Next;
  reinterpret_cast<GenericObject*>(returnNode)->Next = nullptr;

  // Adjust stats
  Statistics_.Allocations_++;
  Statistics_.ObjectsInUse_++;
  Statistics_.FreeObjects_--;
  if (Statistics_.ObjectsInUse_ > Statistics_.MostObjects_)
	  Statistics_.MostObjects_ = Statistics_.ObjectsInUse_;

  // Add the correct signature
  if(Configuration_.DebugOn_)
    memset(returnNode, ALLOCATED_PATTERN, Statistics_.ObjectSize_);

  // If there is a header
  if(Configuration_.HBlockInfo_.type_ != OAConfig::hbNone)
  {
	 // If it's external
  	if(Configuration_.HBlockInfo_.type_ == OAConfig::hbExternal)
		set_external_header(returnNode, label);
	else
		set_header_data(returnNode);

  }

  return returnNode;
}

// Returns an object to the free list for the client (simulates delete)
// Throws an exception if the the object can't be freed. (Invalid object)
void ObjectAllocator::Free(void *Object)
{
	// If they are using the CPPManager
	if (Configuration_.UseCPPMemManager_)
	{
		// Adjust stats
		Statistics_.ObjectsInUse_--;
		Statistics_.FreeObjects_++;
		Statistics_.Deallocations_++;

		// Free the object
		free(Object);

		return;
	}

  // Do error checking
  if (Configuration_.DebugOn_)
  {
    check_within_bounds(Object);
    check_double_free(Object);
    if (Configuration_.PadBytes_ > 0)
      check_corrputed_pad(Object);

	// Add the correct signatures
	memset(Object, FREED_PATTERN, Statistics_.ObjectSize_);
  }
  
  if(Configuration_.HBlockInfo_.type_ != OAConfig::hbNone)
    free_header_data(Object);

  // Put it on the list
  put_on_freelist(Object);

}

// Calls the callback fn for each block still in use
unsigned ObjectAllocator::DumpMemoryInUse(DUMPCALLBACK fn) const
{
  // Walk through the pages
  GenericObject* pageWalker = PageList_;
  // Counts the number of leaks
  unsigned counter = 0;
  
  // While there are pages left
  while(pageWalker != nullptr)
  {
    // For every item on the page
    for(unsigned i = 0; i < Configuration_.ObjectsPerPage_; ++i)
    {
      // Walks through all the objects in a single page
      GenericObject* objectWalker =
	    reinterpret_cast<GenericObject*>(reinterpret_cast<char*>(pageWalker) + sizeof(void*) +
		                                 Configuration_.PadBytes_ + Configuration_.HBlockInfo_.size_
										 + (get_size_of_object() * i));
      
      // If the client still has the object
      if(!is_on_free_list(objectWalker))
      {
        fn(reinterpret_cast<void*>(objectWalker), Statistics_.ObjectSize_);
        
        ++counter;
      }
    }
    
    // Move to the next page
    pageWalker = pageWalker->Next;
  }
  
  return counter;
}

// Calls the callback fn for each block that is potentially corrupted
unsigned ObjectAllocator::ValidatePages(VALIDATECALLBACK fn) const
{
  // Walk through the pages
  GenericObject* pageWalker = PageList_;
  // Counts the number of leaks
  unsigned counter = 0;
  
  // While there are pages left
  while (pageWalker != nullptr)
  {
    // For every item on the page
    for (unsigned i = 0; i < Configuration_.ObjectsPerPage_; ++i)
    {
      // Walks through all the objects in a single page
      void* objectWalker =
	    reinterpret_cast<void*>(reinterpret_cast<char*>(pageWalker) + sizeof(void*) +
		                        Configuration_.PadBytes_ + Configuration_.HBlockInfo_.size_ +
								(get_size_of_object() * i));
      
      // If the client still has the object
      if (is_corrupted(objectWalker))
      {
        fn(reinterpret_cast<void*>(objectWalker), Statistics_.ObjectSize_);
        
        ++counter;
      }
    }
    
    // Move to the next page
    pageWalker = pageWalker->Next;
  }
  
  return counter;
}

// Frees all empty pages (extra credit)
unsigned ObjectAllocator::FreeEmptyPages(void)
{
  return 0;
}

// Returns true if FreeEmptyPages and alignments are implemented
bool ObjectAllocator::ImplementedExtraCredit(void)
{
  return false;
}

/***************************************************************************************************
  Testing/Debugging/Statistic methods
***************************************************************************************************/

// true=enable, false=disable
void ObjectAllocator::SetDebugState(bool State)
{
  Configuration_.DebugOn_ = State;
}

// returns a pointer to the internal free list
const void *ObjectAllocator::GetFreeList(void) const
{
  return reinterpret_cast<void*>(FreeList_);
}

// returns a pointer to the internal page list
const void *ObjectAllocator::GetPageList(void) const
{
  return reinterpret_cast<void*>(PageList_);
}

// returns the configuration parameters
OAConfig ObjectAllocator::GetConfig(void) const
{
  return Configuration_;
}

// returns the statistics for the allocator
OAStats ObjectAllocator::GetStats(void) const
{
  return Statistics_;
}

/***************************************************************************************************
  Private methods
***************************************************************************************************/

// allocates another page of objects
void ObjectAllocator::allocate_new_page()
{
  // Check to see if you need to throw or not
  if (Statistics_.PagesInUse_ != 0 && Statistics_.PagesInUse_ == Configuration_.MaxPages_)
    throw OAException(OAException::E_NO_PAGES, "Max pages have been made!");

  // Allocate the memory
  char *page = nullptr;

  try
  {
    page = new char[Statistics_.PageSize_];
  }
  catch (std::bad_alloc&)
  {
    throw OAException(OAException::E_NO_MEMORY, "No physical memory left!");
  }

  // Set the correct pattern bytes
  if (Configuration_.DebugOn_)
  {
    memset(page, UNALLOCATED_PATTERN, Statistics_.PageSize_);
    set_padding_bytes(page);
    set_header_bytes(page);
  }

  // Set the page to look at the old head and become the new head
  reinterpret_cast<GenericObject*>(page)->Next = PageList_;
  PageList_ = reinterpret_cast<GenericObject*>(page);

  // Adjust the statistics
  Statistics_.PagesInUse_++;

  // Space the pointers on the new page
  allocate_objects(page);
}

void ObjectAllocator::allocate_objects(char *page)
{
  // For every object to be on the page
  for(unsigned i = 0; i < Configuration_.ObjectsPerPage_; ++i)
  {
    // The beginning of each object
    GenericObject* node =
	  reinterpret_cast<GenericObject*>(page + get_size_of_header() + (get_size_of_object() * i) +
	                                   Configuration_.HBlockInfo_.size_ + Configuration_.PadBytes_);

    // Adjust the next pointer correctly
    node->Next = FreeList_;
	FreeList_ = node;

  }

  // Adjust stats
  Statistics_.FreeObjects_ += Configuration_.ObjectsPerPage_;

}

// Puts Object onto the free list
void ObjectAllocator::put_on_freelist(void* Object)
{
  // Have the object point to the head of the free list
  reinterpret_cast<GenericObject*>(Object)->Next = FreeList_;

  // Reset the head of the list
  FreeList_ = reinterpret_cast<GenericObject*>(Object);

  // Then, adjust stats
  Statistics_.ObjectsInUse_--;
  Statistics_.FreeObjects_++;
  Statistics_.Deallocations_++;
}

void ObjectAllocator::check_double_free(void* Object)
{
  // Check if it's been freed already
  GenericObject* walker = FreeList_;
  
  // Walking through the list
  while (walker != nullptr)
  {
  	// If the two objects match
  	if (walker == reinterpret_cast<GenericObject*>(Object))
  	{
		throw OAException(OAException::E_MULTIPLE_FREE, "Object has already been freed!");
  	}
  
	walker = walker->Next;
  }

}

void ObjectAllocator::check_within_bounds(void* Object)
{
  // This is called with one object that is trying to be freed

  // Walk through the Page List
  GenericObject* walker = PageList_;

  while(walker != nullptr)
  {
	  // If the beginning of the object is after a page beginning
	  if((reinterpret_cast<char*>(Object) > reinterpret_cast<char*>(walker)) &&
	     (reinterpret_cast<char*>(Object) < reinterpret_cast<char*>(walker) + Statistics_.PageSize_))
	  {
		  // if the end of the object is before the end of the same page
		  size_t test1 = reinterpret_cast<size_t>(Object) - reinterpret_cast<size_t>(walker) -
		                 get_size_of_header() - Configuration_.PadBytes_ -
						 Configuration_.HBlockInfo_.size_;
		  size_t test2 = Statistics_.ObjectSize_ + (Configuration_.PadBytes_ * 2) +
		                 Configuration_.HBlockInfo_.size_;
						 
		  if (!(test1 % test2))
			  return;

		  break;
	  }
	  
	  walker = walker->Next;
  }
  
  // The item was never within a single page
  throw OAException(OAException::E_BAD_BOUNDARY, "Object is not on a page correctly!");

}

void ObjectAllocator::check_corrputed_pad(void* Object)
{
  // This will check the first padding
  unsigned char *leftPad = reinterpret_cast<unsigned char*>(reinterpret_cast<char*>(Object) -
                                                            Configuration_.PadBytes_);
  // This will check the second padding
  unsigned char *rightPad = reinterpret_cast<unsigned char*>(reinterpret_cast<char*>(Object) +
                                                             Statistics_.ObjectSize_);

  // Check if the bytes have been touched
  for(unsigned i = 0; i < Configuration_.PadBytes_; ++i)
  {
    // If this char doesn't have the pattern
    if(*leftPad != PAD_PATTERN)
    {
      throw OAException(OAException::E_CORRUPTED_BLOCK, "The boundaries of this object were corrupted!");
    }
    // If this char doesn't have the pattern
    if (*rightPad != PAD_PATTERN)
    {
      throw OAException(OAException::E_CORRUPTED_BLOCK, "The boundaries of this object were corrupted!");
    }
    
    // Move each pad checker forward 1 char
    ++leftPad;
    ++rightPad;
  }

}

void ObjectAllocator::set_padding_bytes(char* Page)
{
  // Walks through the page
  char* walker = reinterpret_cast<char*>(Page + get_size_of_header() + Configuration_.PadBytes_ +
                                         Configuration_.HBlockInfo_.size_);

  // While in the list
  for(unsigned i = 0; i < Configuration_.ObjectsPerPage_; ++i)
  {
    // Set pad bytes behind the object
    memset((walker - Configuration_.PadBytes_), PAD_PATTERN, Configuration_.PadBytes_);

    // Set pad bytes ahead the object
    memset((walker + Statistics_.ObjectSize_), PAD_PATTERN, Configuration_.PadBytes_);

    // Move to the next object
    walker += get_size_of_object();
  }

}

void ObjectAllocator::set_header_bytes(char* Page)
{
  // Walks through the page
  char* walker = reinterpret_cast<char*>(Page + get_size_of_header() +
                                         Configuration_.HBlockInfo_.size_ + Configuration_.PadBytes_);
  
  // While in the list
  for (unsigned i = 0; i < Configuration_.ObjectsPerPage_; ++i)
  {
    // If the header is external
    if(Configuration_.HBlockInfo_.type_ == OAConfig::hbExternal)
    {
      // Make a mem block info pointer
	  reinterpret_cast<GenericObject*>(walker - Configuration_.HBlockInfo_.size_ -
	                                   Configuration_.PadBytes_)->Next = nullptr;
    }
	else
	{
      // Set header for the object
      memset((walker - Configuration_.PadBytes_ - Configuration_.HBlockInfo_.size_), 0,
	         Configuration_.HBlockInfo_.size_);
	}
    
	// Move to the next object
	walker += get_size_of_object();
  }
}

void ObjectAllocator::set_header_data(void* Object)
{
  // Set the "free" bit
  bool *freeBit = reinterpret_cast<bool*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
                                          - sizeof(bool));
  *freeBit = true;

  // Set the allocation number
  int *allocations = reinterpret_cast<int*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
                                            - sizeof(int) - sizeof(bool));
  *allocations = Statistics_.Allocations_;

  // If it's more than basic
  if(Configuration_.HBlockInfo_.type_ != OAConfig::hbBasic)
  {
	  // Set the counter bits
	  short *counter = reinterpret_cast<short*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
	                                            - sizeof(int) - sizeof(bool) - sizeof(short));
	  ++(*counter);

	  // Clear the user's data
	  void *userData = reinterpret_cast<bool*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
	                                           - Configuration_.HBlockInfo_.size_);
	  memset(userData, 0, (Configuration_.HBlockInfo_.size_ - sizeof(int) - sizeof(bool) - sizeof(short)));
  }

}

void ObjectAllocator::set_external_header(void* Object, const char* label)
{
  // Create the new external header
  MemBlockInfo *newInfo = new MemBlockInfo(true, label, Statistics_.Allocations_);

  // Point at it
  intptr_t* pointing = reinterpret_cast<intptr_t*>(reinterpret_cast<char*>(Object) -
                                                   Configuration_.PadBytes_ -
												   Configuration_.HBlockInfo_.size_);
  intptr_t address = reinterpret_cast<intptr_t>(newInfo);
  *pointing = address;
}

void ObjectAllocator::free_header_data(void* Object)
{
	// If the header is external
	if(Configuration_.HBlockInfo_.type_ == OAConfig::hbExternal)
	{
      // Get the pointer
	  MemBlockInfo **externalHeader = reinterpret_cast<MemBlockInfo**>(reinterpret_cast<char*>(Object)
	                                                                   - Configuration_.PadBytes_ -
																	   Configuration_.HBlockInfo_.size_);
	  
	  // Delete this header
	  delete *externalHeader;
	  
	  // Set the value of the pointer in the object to nullptr
	  *externalHeader = nullptr;
	}
	else
	{
      // In case the header is Extended
      short counter = 0;
      
      // Save the counter value
      if (Configuration_.HBlockInfo_.type_ == OAConfig::hbExtended)
        counter = *reinterpret_cast<short*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
	                                        - sizeof(int) - sizeof(bool) - sizeof(short));
      
      // Clear everything
      memset((reinterpret_cast<char*>(Object) - Configuration_.PadBytes_ - Configuration_.HBlockInfo_.size_),
	         0, Configuration_.HBlockInfo_.size_);
      
      // If the header was Extended
      if (Configuration_.HBlockInfo_.type_ == OAConfig::hbExtended)
      {
        short *newCounter = reinterpret_cast<short*>(reinterpret_cast<char*>(Object) - Configuration_.PadBytes_
		                                             - sizeof(int) - sizeof(bool) - sizeof(short));
        *newCounter = counter;
      }
	}

}

size_t ObjectAllocator::get_size_of_header()
{
  // This replaces using size of the void pointer
  return sizeof(void*);
}

size_t ObjectAllocator::get_size_of_object() const
{
  // This will cover the full distance of one object
  return Statistics_.ObjectSize_ + (Configuration_.PadBytes_ * 2) + Configuration_.HBlockInfo_.size_;
}

bool ObjectAllocator::is_on_free_list(void* Object) const
{
	// Checks against all objects in the free list
	GenericObject* walker = FreeList_;

	// Walking through the list
	while (walker != nullptr)
	{
		// If the two objects match
		if (walker == reinterpret_cast<GenericObject*>(Object))
		{
			// The data is on the free list
			return true;
		}

		// Move to the next free item
		walker = walker->Next;
	}

	// The client has the data
	return false;
}

bool ObjectAllocator::is_corrupted(void* Object) const
{
	// This will check the first padding
	unsigned char *leftPad = reinterpret_cast<unsigned char*>(reinterpret_cast<char*>(Object)
	                                                          - Configuration_.PadBytes_);
	// This will check the second padding
	unsigned char *rightPad = reinterpret_cast<unsigned char*>(reinterpret_cast<char*>(Object)
	                                                           + Statistics_.ObjectSize_);

	// Check if the bytes have been touched
	for (unsigned i = 0; i < Configuration_.PadBytes_; ++i)
	{
		// If this char doesn't have the pattern
		if (*leftPad != PAD_PATTERN)
		{
			// The object is corrupted
			return true;
		}
		// If this char doesn't have the pattern
		if (*rightPad != PAD_PATTERN)
		{
			// The object is corrupted
			return true;
		}

		// Move each pad checker forward 1 char
		++leftPad;
		++rightPad;
	}

	// The object is not corrupted
	return false;
}

