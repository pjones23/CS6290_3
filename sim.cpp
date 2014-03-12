#include "common.h"
#include "sim.h"
#include "trace.h" 
#include "cache.h"  /**** NEW-LAB2*/ 
#include "memory.h" // NEW-LAB2
#include "bpred.h"
#include "vmem.h"
#include <stdlib.h>
#include <ctype.h> /* Library for useful character operations */
/*******************************************************************/
/* Simulator frame */
/*******************************************************************/

bool run_a_cycle(memory_c *m); // please modify run_a_cycle function argument  /** NEW-LAB2 */
void init_structures(memory_c *m); // please modify init_structures function argument  /** NEW-LAB2 */ 

/* uop_pool related variables */

uint32_t free_op_num;
uint32_t active_op_num;
Op *op_pool;
Op *op_pool_free_head = NULL;

/* simulator related local functions */

bool icache_access(ADDRINT addr);
bool dcache_access(ADDRINT addr);
void init_latches(void);
uint64_t virtualToPhysical(ADDRINT vaddr, uint64_t pfn);
uint64_t getPTE(ADDRINT vaddr);
uint64_t getVPN(ADDRINT vaddr);
void installInTLB(void);
list<Op*> TLBinstallQueue;
bool TLBMSHRstall;
bool dcacheTLBstall;
bool memOpsStall;

#include "knob.h"
#include "all_knobs.h"

// knob variables
KnobsContainer *g_knobsContainer; /* < knob container > */
all_knobs_c *g_knobs; /* < all knob variables > */

gzFile g_stream;

void init_knobs(int argc, char** argv) {
	// Create the knob managing class
	g_knobsContainer = new KnobsContainer();

	// Get a reference to the actual knobs for this component instance
	g_knobs = g_knobsContainer->getAllKnobs();

	// apply the supplied command line switches
	char* pInvalidArgument = NULL;
	g_knobsContainer->applyComandLineArguments(argc, argv, &pInvalidArgument);

	g_knobs->display();
}

void read_trace_file(void) {
	g_stream = gzopen((KNOB(KNOB_TRACE_FILE)->getValue()).c_str(), "r");
}

// simulator main function is called from outside of this file 

void simulator_main(int argc, char** argv) {
	init_knobs(argc, argv);

	// trace driven simulation
	read_trace_file();

	/** NEW-LAB2 *//* just note: passing main memory pointers is hack to mix up c++ objects and c-style code *//* Not recommended at all */
	memory_c *main_memory = new memory_c();  // /** NEW-LAB2 */

	init_structures(main_memory); // please modify run_a_cycle function argument  /** NEW-LAB2 */
	run_a_cycle(main_memory); // please modify run_a_cycle function argument  /** NEW-LAB2 */

}
int op_latency[NUM_OP_TYPE];

void init_op_latency(void) {
	op_latency[OP_INV] = 1;
	op_latency[OP_NOP] = 1;
	op_latency[OP_CF] = 1;
	op_latency[OP_CMOV] = 1;
	op_latency[OP_LDA] = 1;
	op_latency[OP_LD] = 1;
	op_latency[OP_ST] = 1;
	op_latency[OP_IADD] = 1;
	op_latency[OP_IMUL] = 2;
	op_latency[OP_IDIV] = 4;
	op_latency[OP_ICMP] = 2;
	op_latency[OP_LOGIC] = 1;
	op_latency[OP_SHIFT] = 2;
	op_latency[OP_BYTE] = 1;
	op_latency[OP_MM] = 2;
	op_latency[OP_FMEM] = 2;
	op_latency[OP_FCF] = 1;
	op_latency[OP_FCVT] = 4;
	op_latency[OP_FADD] = 2;
	op_latency[OP_FMUL] = 4;
	op_latency[OP_FDIV] = 16;
	op_latency[OP_FCMP] = 2;
	op_latency[OP_FBIT] = 2;
	op_latency[OP_FCMP] = 2;
}

void init_op(Op *op) {
	op->num_src = 0;
	op->src[0] = -1;
	op->src[1] = -1;
	op->dst = -1;
	op->opcode = 0;
	op->is_fp = false;
	op->cf_type = NOT_CF;
	op->mem_type = NOT_MEM;
	op->write_flag = 0;
	op->inst_size = 0;
	op->ld_vaddr = 0;
	op->st_vaddr = 0;
	op->instruction_addr = 0;
	op->branch_target = 0;
	op->actually_taken = 0;
	op->mem_read_size = 0;
	op->mem_write_size = 0;
	op->valid = FALSE;
	/* you might add more features here */
	op->mispredictedBranch = FALSE;
	op->TLBmissOp = false;
}

void init_op_pool(void) {
	/* initialize op pool */
	op_pool = new Op[1024];
	free_op_num = 1024;
	active_op_num = 0;
	uint32_t op_pool_entries = 0;
	int ii;
	for (ii = 0; ii < 1023; ii++) {

		op_pool[ii].op_pool_next = &op_pool[ii + 1];
		op_pool[ii].op_pool_id = op_pool_entries++;
		init_op(&op_pool[ii]);
	}
	op_pool[ii].op_pool_next = op_pool_free_head;
	op_pool[ii].op_pool_id = op_pool_entries++;
	init_op(&op_pool[ii]);
	op_pool_free_head = &op_pool[0];
}

Op *get_free_op(void) {
	/* return a free op from op pool */

	if (op_pool_free_head == NULL || (free_op_num == 1)) {
		std::cout << "ERROR! OP_POOL SIZE is too small!! " << endl;
		std::cout << "please check free_op function " << endl;
		assert(1);
		exit(1);
	}

	free_op_num--;
	assert(free_op_num);

	Op *new_op = op_pool_free_head;
	op_pool_free_head = new_op->op_pool_next;
	assert(!new_op->valid);
	init_op(new_op);
	active_op_num++;
	return new_op;
}

void free_op(Op *op) {
	free_op_num++;
	active_op_num--;
	op->valid = FALSE;
	op->op_pool_next = op_pool_free_head;
	op_pool_free_head = op;
}

/*******************************************************************/
/*  Data structure */
/*******************************************************************/

typedef struct pipeline_latch_struct {
	Op *op; /* you must update this data structure. */
	list<Op*> op_queue;	//for MEM latch multiple instruction retirement
	bool op_valid;
	bool stage_stall;//added another variable to latch to understand if there is a stall
	/* you might add more data structures. But you should complete the above data elements */
} pipeline_latch;

typedef struct Reg_element_struct {
	bool valid;
	// data is not needed
	/* you might add more data structures. But you should complete the above data elements */
} REG_element;

REG_element register_file[NUM_REG];
void init_register_file();
bool pipeline_latches_empty();

/*******************************************************************/
/* These are the functions you'll have to write.  */
/*******************************************************************/

void FE_stage();
void ID_stage();
void EX_stage();
void MEM_stage(memory_c *main_memory); // please modify MEM_stage function argument  /** NEW-LAB2 */ 
void WB_stage();

/*******************************************************************/
/*  These are the variables you'll have to write.  */
/*******************************************************************/

bool sim_end_condition = FALSE; /* please complete the condition. */
UINT64 retired_instruction = 0; /* number of retired instruction. (only correct instructions) */
UINT64 cycle_count = 0; /* total number of cycles */
UINT64 data_hazard_count = 0;
UINT64 control_hazard_count = 0;
UINT64 icache_miss_count = 0; /* total number of icache misses. for Lab #2 and Lab #3 */
UINT64 dcache_hit_count = 0; /* total number of dcache  misses. for Lab #2 and Lab #3 */
UINT64 dcache_miss_count = 0; /* total number of dcache  misses. for Lab #2 and Lab #3 */
UINT64 l2_cache_miss_count = 0; /* total number of L2 cache  misses. for Lab #2 and Lab #3 */
UINT64 dram_row_buffer_hit_count = 0; /* total number of dram row buffer hit. for Lab #2 and Lab #3 */ // NEW-LAB2
UINT64 dram_row_buffer_miss_count = 0; /* total number of dram row buffer hit. for Lab #2 and Lab #3 */ // NEW-LAB2
UINT64 store_load_forwarding_count = 0; /* total number of store load forwarding for Lab #2 and Lab #3 */ // NEW-LAB2
UINT64 store_store_forwarding_count = 0; /* total number of store load forwarding for Lab #2 and Lab #3 */ // NEW-LAB2

uint64_t bpred_mispred_count = 0; /* total number of branch mispredictions */ // NEW-LAB3
uint64_t bpred_okpred_count = 0; /* total number of correct branch predictions */ // NEW-LAB3
uint64_t dtlb_hit_count = 0; /* total number of DTLB hits */ // NEW-LAB3
uint64_t dtlb_miss_count = 0; /* total number of DTLB miss */ // NEW-LAB3

bpred *branchpred; // NEW-LAB3 (student need to initialize)
tlb *dtlb;        // NEW-LAB3 (student need to intialize)

pipeline_latch *MEM_latch;
pipeline_latch *EX_latch;
pipeline_latch *ID_latch;
pipeline_latch *FE_latch;
UINT64 ld_st_buffer[LD_ST_BUFFER_SIZE]; /* this structure is deprecated. do not use */
UINT64 next_pc;

Cache *data_cache = new Cache;  // NEW-LAB2

//New variables added by apkarande
UINT64 reg_writing_ops[NUM_REG];
UINT64 last_inst_id = 0;
bool trace_over = false;

/*******************************************************************/
/*  Print messages  */
/*******************************************************************/
void print_stats() {
	std::ofstream out((KNOB(KNOB_OUTPUT_FILE)->getValue()).c_str());
	/* Do not modify this function. This messages will be used for grading */
	out << "Total instruction: " << retired_instruction << endl;
	out << "Total cycles: " << cycle_count << endl;
	float ipc = (
			cycle_count ?
					((float) retired_instruction / (float) cycle_count) : 0);
	out << "IPC: " << ipc << endl;
	out << "Total I-cache miss: " << icache_miss_count << endl;
	out << "Total D-cache miss: " << dcache_miss_count << endl;
	out << "Total D-cache hit: " << dcache_hit_count << endl;
	out << "Total data hazard: " << data_hazard_count << endl;
	out << "Total control hazard : " << control_hazard_count << endl;
	out << "Total DRAM ROW BUFFER Hit: " << dram_row_buffer_hit_count << endl;
	out << "Total DRAM ROW BUFFER Miss: " << dram_row_buffer_miss_count << endl;
	out << "Total Store-load forwarding: " << store_load_forwarding_count
			<< endl;

	// new for LAB3
	out << "Total Branch Predictor Mispredictions: " << bpred_mispred_count
			<< endl;
	out << "Total Branch Predictor OK predictions: " << bpred_okpred_count
			<< endl;
	out << "Total DTLB Hit: " << dtlb_hit_count << endl;
	out << "Total DTLB Miss: " << dtlb_miss_count << endl;

	out.close();
}

/*******************************************************************/

/*******************************************************************/
/*  Support Functions  */
/*******************************************************************/

bool get_op(Op *op) {
	static UINT64 unique_count = 0;
	Trace_op trace_op;
	bool success = FALSE;
	// read trace
	// fill out op info
	// return FALSE if the end of trace
	//success = (gzread(g_stream, &trace_op, sizeof(Trace_op)) >0 );
	success =
			(gzread(g_stream, &trace_op, sizeof(Trace_op)) == sizeof(Trace_op));

	/* copy trace structure to op */
	if (success) {
		if (KNOB(KNOB_PRINT_INST)->getValue())
			dprint_trace(&trace_op);
		copy_trace_op(&trace_op, op);

		op->inst_id = unique_count++;
		op->valid = TRUE;
		last_inst_id = op->inst_id + 1;
	} else {
		if (unique_count == 0) {
			//cout << "Error in trace file" << endl;
			exit(0);
		}
		//last_inst_id = unique_count - 1;
		trace_over = true;
	}

	return success;
}
/* return op execution cycle latency */

int get_op_latency(Op *op) {
	if (op->opcode >= NUM_OP_TYPE) {
		cout << "opcode " << (int) op->opcode << endl;
		cout << "inst id" << (int) op->inst_id << endl;
	}
	assert(op->opcode < NUM_OP_TYPE);
	return op_latency[op->opcode];
}

/* Print out all the register values */
void dump_reg() {
	for (int ii = 0; ii < NUM_REG; ii++) {
		std::cout << cycle_count << ":register[" << ii << "]: V:"
				<< register_file[ii].valid << endl;
	}
}

void print_pipeline() {
	std::cout << "--------------------------------------------" << endl;
	std::cout << "cycle count : " << dec << cycle_count
			<< " retired_instruction : " << retired_instruction << endl;
	std::cout << (int) cycle_count << " FE: ";
	if (FE_latch->op_valid) {
		Op *op = FE_latch->op;
		cout << (int) op->inst_id;
	} else {
		cout << "####";
	}
	std::cout << " ID: ";
	if (ID_latch->op_valid) {
		Op *op = ID_latch->op;
		cout << (int) op->inst_id;
	} else {
		cout << "####";
	}
	std::cout << " EX: ";
	if (EX_latch->op_valid) {
		Op *op = EX_latch->op;
		cout << (int) op->inst_id;
	} else {
		cout << "####";
	}

	std::cout << " MEM: ";
	if (MEM_latch->op_valid || (!temp_mem_queue.empty())) {
		if (MEM_latch->op_valid) {
			list<Op *>::iterator cii;
			for (cii = MEM_latch->op_queue.begin();
					cii != MEM_latch->op_queue.end(); cii++) {
				Op *entry = (*cii);
				cout << " " << (int) (entry->inst_id);
			}
		}
		if (!temp_mem_queue.empty()) {
			list<Op *>::iterator cii;
			for (cii = temp_mem_queue.begin(); cii != temp_mem_queue.end();
					cii++) {
				Op *entry = (*cii);
				cout << " " << (int) (entry->inst_id);
			}
		}
	} else {
		cout << "####";
	}
	cout << endl;
	//  dump_reg();
	std::cout << "--------------------------------------------" << endl;
}

void print_heartbeat() {
	static uint64_t last_cycle;
	static uint64_t last_inst_count;
	float temp_ipc = float(retired_instruction - last_inst_count)
			/ (float) (cycle_count - last_cycle);
	float ipc = float(retired_instruction) / (float) (cycle_count);
	/* Do not modify this function. This messages will be used for grading */
	cout << "**Heartbeat** cycle_count: " << cycle_count << " inst:"
			<< retired_instruction << " IPC: " << temp_ipc << " Overall IPC: "
			<< ipc << endl;
	last_cycle = cycle_count;
	last_inst_count = retired_instruction;
}
/*******************************************************************/
/*                                                                 */
/*******************************************************************/

bool run_a_cycle(memory_c *main_memory) {
	long int retired_instruction_cpy = 0;
	int terminate_count = 0;
	bool terminate_program = false;
	for (;;) {
		if ((KNOB(KNOB_MAX_SIM_COUNT)->getValue()
				&& (cycle_count >= KNOB(KNOB_MAX_SIM_COUNT)->getValue()))
				|| (KNOB(KNOB_MAX_INST_COUNT)->getValue()
						&& (retired_instruction
								>= KNOB(KNOB_MAX_INST_COUNT)->getValue()))
				|| (sim_end_condition) || terminate_program) {
			// please complete sim_end_condition
			// finish the simulation
			print_heartbeat();
			print_stats();
			return TRUE;
		}
		cycle_count++;
		if (!(cycle_count % 5000)) {
			print_heartbeat();
		}

		/*section to terminate the program if it fails to exit normally*/
		if (terminate_count > 1000) {
			if (retired_instruction_cpy == retired_instruction)
				terminate_program = true;
			else {
				retired_instruction_cpy = retired_instruction;
				terminate_count = 0;
			}
		} else
			terminate_count++;
		/*section to terminate the program if it fails to exit normally*/

		main_memory->run_a_cycle();          // *NEW-LAB2

		WB_stage();
		MEM_stage(main_memory); // please modify MEM_stage function argument  /** NEW-LAB2 */
		EX_stage();
		ID_stage();
		FE_stage();
		//install ready TLB entries
		installInTLB();

		if (trace_over && main_memory->all_mem_structures_empty()
				&& pipeline_latches_empty())
			sim_end_condition = true;
		if (KNOB(KNOB_PRINT_PIPE_FREQ)->getValue()
				&& !(cycle_count % KNOB(KNOB_PRINT_PIPE_FREQ)->getValue()))
			print_pipeline();
	}
	return TRUE;
}

/*******************************************************************/
/* Complete the following fuctions.  */
/* You can add new data structures and also new elements to Op, Pipeline_latch data structure */
/*******************************************************************/

void init_structures(memory_c *main_memory) // please modify init_structures function argument  /** NEW-LAB2 */ 
		{
	init_op_pool();
	init_op_latency();
	/* please initialize other data stucturs */
	/* you must complete the function */
	init_latches();
	init_register_file();	//fn added by apkarande
	main_memory->init_mem();	//initialize main memory
	cache_init(data_cache, KNOB(KNOB_DCACHE_SIZE)->getValue(),
	KNOB(KNOB_BLOCK_SIZE)->getValue(),
	KNOB(KNOB_DCACHE_WAY)->getValue(), "L1 dcache");	//initialize data cache
	branchpred = bpred_new((bpred_type) (KNOB(KNOB_BPRED_TYPE)->getValue()),
	KNOB(KNOB_BPRED_HIST_LEN)->getValue()); //initialize branch predictor

	dtlb = tlb_new(KNOB(KNOB_TLB_ENTRIES)->getValue()); //size is knob
	TLBMSHRstall = false;
	dcacheTLBstall = false;
	memOpsStall = false;
}

void WB_stage() {
	/* You MUST call free_op function here after an op is retired */
	/* you must complete the function */
	while (!MEM_latch->op_queue.empty()) {
		Op *retire_op = MEM_latch->op_queue.front();
		//check if using branch predictor check if there was misprediction
		if (KNOB(KNOB_USE_BPRED)->getValue() == TRUE) {
			if (retire_op->cf_type
					== CF_CBR&& retire_op->mispredictedBranch == TRUE) {
				FE_latch->stage_stall = false;
			}
		} else {
			//add else to stall for all branch
			if (retire_op->cf_type >= CF_BR) {
				FE_latch->stage_stall = false;
			}
		}
		if ((retire_op->dst != -1)
				&& (reg_writing_ops[retire_op->dst] == retire_op->inst_id))
			register_file[retire_op->dst].valid = true;
		retired_instruction++;
		MEM_latch->op_queue.pop_front();
		free_op(retire_op);

		//if(retire_op->inst_id == last_inst_id)
		//sim_end_condition = true;
	}
	MEM_latch->op_valid = false;		//not used
}

void MEM_stage(memory_c *main_memory) // please modify MEM_stage function argument  /** NEW-LAB2 */
		{
	/* you must complete the function */
	static int latency_count;
	static bool mshr_full = false;
	if (EX_latch->op_valid == true) {
		//std::cout << "TLB MSHR stall: " << TLBMSHRstall << endl;
		//std::cout << "Mem ops stall: " << memOpsStall << endl;
		//std::cout << "TLB dcache stall: " << dcacheTLBstall << endl;

		//memory translation
		bool TLBhit = false;
		if (KNOB(KNOB_ENABLE_VMEM)->getValue() == TRUE && TLBMSHRstall == false
				&& memOpsStall == false) {
			if ((EX_latch->op)->mem_type == MEM_LD) {

				uint64_t pfn = 0;
				uint64_t vpn = getVPN((EX_latch->op)->ld_vaddr);

				if (dcacheTLBstall == false && TLBMSHRstall == false) {
					TLBhit = tlb_access(dtlb, vpn, 0, &pfn);
					if (TLBhit == TRUE) {
						if ((EX_latch->op)->TLBmissOp == false) {
							dtlb_hit_count++;
							//std::cout << "TLB hit: inst_id "
								//	<< (EX_latch->op)->inst_id << endl;
						}
					} else {
						dtlb_miss_count++;
						(EX_latch->op)->TLBmissOp = true;
						//std::cout << "TLB miss: inst_id "
							//	<< (EX_latch->op)->inst_id << endl;
					}
				} else {
					TLBhit = false;
				}

				if (TLBhit == TRUE) {
					(EX_latch->op)->ld_vaddr = virtualToPhysical(
							(EX_latch->op)->ld_vaddr, pfn);
				} else { //if TLB miss
					uint64_t pteaddr = getPTE((EX_latch->op)->ld_vaddr);

					if (mshr_full == false) {
						if (latency_count == 0) {
							latency_count =
							KNOB(KNOB_DCACHE_LATENCY)->getValue();
							EX_latch->stage_stall = true;
							dcacheTLBstall = true;
						}
						latency_count--;
						if (latency_count == 0) {
							dcacheTLBstall = false;
							if (dcache_access(pteaddr)) {
								dcache_hit_count++;
								pfn = vmem_vpn_to_pfn(vpn, 0);
								tlb_install(dtlb, vpn, 0, pfn);
								//std::cout << "dcache hit" << endl;

							} else { // if dcache miss
								dcache_miss_count++;
								//std::cout << "dcache miss" << endl;
								Op *dummyOp = new Op();
								dummyOp->opcode = OP_DUMMY;
								dummyOp->mem_type = MEM_LD;
								dummyOp->ld_vaddr = pteaddr;
								if (main_memory->insert_mshr(dummyOp) == true) { // try to insert into mshr
									EX_latch->stage_stall = true; // stall until the request returns from the mshr
									//EX_latch->op_valid = false; // using this to stop execution of mem stage until request returns from the mshr
									mshr_full = false;
									TLBMSHRstall = true;
									//std::cout << "sent to MSHR (waiting)"
										//	<< endl;
								} else {
									EX_latch->stage_stall = true; // if mshr is full
									mshr_full = true;
									//std::cout << "MSHR is full (waiting)"
										//	<< endl;
								}
							}
						}
					} else { // if mshr is full
						Op *dummyOp = new Op();
						dummyOp->opcode = OP_DUMMY;
						dummyOp->mem_type = MEM_LD;
						dummyOp->ld_vaddr = pteaddr;
						if (main_memory->insert_mshr(dummyOp) == true) // try to insert into mshr
								{
							EX_latch->stage_stall = true; // stall until the request returns from the mshr
							//EX_latch->op_valid = false; // using this to stop execution of mem stage until request returns from the mshr
							mshr_full = false;
							TLBMSHRstall = true;
							//std::cout << "sent to MSHR (waiting)" << endl;
						} else {
							EX_latch->stage_stall = true;	// if mshr is full
							mshr_full = true;
							//std::cout << "MSHR is full (waiting)" << endl;
						}
					}

				} // end tlb miss stuff for MEM_LD

			} else if ((EX_latch->op)->mem_type == MEM_ST) {
				//replicate ld, but replace with st

				uint64_t pfn = 0;
				uint64_t vpn = getVPN((EX_latch->op)->st_vaddr);

				if (dcacheTLBstall == false && TLBMSHRstall == false) {
					TLBhit = tlb_access(dtlb, vpn, 0, &pfn);
					if (TLBhit == TRUE) {
						if ((EX_latch->op)->TLBmissOp == false) {
							dtlb_hit_count++;
							//std::cout << "TLB hit: " << (EX_latch->op)->inst_id
								//	<< endl;
						}
					} else {
						dtlb_miss_count++;
						(EX_latch->op)->TLBmissOp = true;
						//std::cout << "TLB miss: " << (EX_latch->op)->inst_id
							//	<< endl;
					}
				} else {
					TLBhit = false;
				}

				if (TLBhit == TRUE) {
					(EX_latch->op)->st_vaddr = virtualToPhysical(
							(EX_latch->op)->st_vaddr, pfn);
				} else { //if TLB miss
					uint64_t pteaddr = getPTE((EX_latch->op)->st_vaddr);

					if (mshr_full == false) {
						if (latency_count == 0) {
							latency_count =
							KNOB(KNOB_DCACHE_LATENCY)->getValue();
							EX_latch->stage_stall = true;
							dcacheTLBstall = true;
						}
						latency_count--;
						if (latency_count == 0) {
							dcacheTLBstall = false;
							if (dcache_access(pteaddr)) {
								dcache_hit_count++;
								pfn = vmem_vpn_to_pfn(vpn, 0);
								tlb_install(dtlb, vpn, 0, pfn);
								//std::cout << "dcache hit" << endl;

							} else { // if dcache miss
								dcache_miss_count++;
								//std::cout << "dcache miss" << endl;
								Op *dummyOp = new Op();
								dummyOp->opcode = OP_DUMMY;
								dummyOp->mem_type = MEM_LD;
								dummyOp->ld_vaddr = pteaddr;
								if (main_memory->insert_mshr(dummyOp) == true) { // try to insert into mshr
									EX_latch->stage_stall = true; // stall until the request returns from the mshr
									//EX_latch->op_valid = false; // using this to stop execution of mem stage until request returns from the mshr
									mshr_full = false;
									TLBMSHRstall = true;
									//std::cout << "sent to MSHR (waiting)"
										//	<< endl;
								} else {
									EX_latch->stage_stall = true; // if mshr is full
									mshr_full = true;
									//std::cout << "MSHR is full (waiting)"
										//	<< endl;
								}
							}
						}
					} else { // if mshr is full
						Op *dummyOp = new Op();
						dummyOp->opcode = OP_DUMMY;
						dummyOp->mem_type = MEM_LD;
						dummyOp->ld_vaddr = pteaddr;
						if (main_memory->insert_mshr(dummyOp) == true) // try to insert into mshr
								{
							EX_latch->stage_stall = true; // stall until the request returns from the mshr
							//EX_latch->op_valid = false; // using this to stop execution of mem stage until request returns from the mshr
							mshr_full = false;
							TLBMSHRstall = true;
							//std::cout << "sent to MSHR (waiting)" << endl;
						} else {
							EX_latch->stage_stall = true;	// if mshr is full
							mshr_full = true;
							//std::cout << "MSHR is full (waiting)" << endl;
						}
					}

				} // end tlb miss stuff for MEM_ST

			} else {
				// set to false since there was no access to the TLB and allow
				// the lab2 code to correctly move the op to the next stage
				TLBhit = TRUE;
			}
		} // end if (KNOB(KNOB_ENABLE_VMEM)->getValue() == TRUE)

		// memory operations
		if ((KNOB(KNOB_ENABLE_VMEM)->getValue() == FALSE)
				|| (KNOB(KNOB_ENABLE_VMEM)->getValue() == TRUE && TLBhit == TRUE
						&& TLBMSHRstall == false)
				|| (KNOB(KNOB_ENABLE_VMEM)->getValue() == TRUE
						&& memOpsStall == true)) {

			/* Do everything*/
			if ((EX_latch->op)->mem_type == MEM_LD) {
				if (mshr_full == false) {
					if (latency_count == 0) {
						latency_count = KNOB(KNOB_DCACHE_LATENCY)->getValue();
						EX_latch->stage_stall = true;
						memOpsStall = true; // stop access to memory translation
					}
					latency_count--;
					if (latency_count == 0) {
						if (dcache_access((EX_latch->op)->ld_vaddr)) {
							dcache_hit_count++;
							EX_latch->op_valid = false;
							EX_latch->stage_stall = false;

							memOpsStall = false; // allows for memory translation for new op after this one gets sent through

							fill_retire_queue(EX_latch->op); //not really a broadcast, just using available function
						} else {
							dcache_miss_count++;
							if (main_memory->store_load_forwarding(
									EX_latch->op)) //check if store load fwding is possible
									{
								store_load_forwarding_count++;
								EX_latch->op_valid = false;
								EX_latch->stage_stall = false;

								memOpsStall = false; // allows for memory translation for new op after this one gets sent through

								fill_retire_queue(EX_latch->op); //not really a broadcast, just using available function
							} else if (main_memory->check_piggyback(
									EX_latch->op) == false) {
								if (main_memory->insert_mshr(EX_latch->op)
										== true) //if cannot be piggybacked, try inserting a new entry
										{
									EX_latch->op_valid = false;
									EX_latch->stage_stall = false;
									mshr_full = false;

									memOpsStall = false; // allows for memory translation for new op after this one gets sent through
								} else {
									EX_latch->stage_stall = true; //if mshr is full
									mshr_full = true;
									memOpsStall = true; // stop access to memory translation
								}
							} else {
								EX_latch->op_valid = false;
								EX_latch->stage_stall = false;
								memOpsStall = false; // stop access to memory translation
							}
						}
					}
				} else {
					if (main_memory->insert_mshr(EX_latch->op) == true)	//if cannot be piggybacked, try inserting a new entry
							{
						EX_latch->op_valid = false;
						EX_latch->stage_stall = false;
						mshr_full = false;

						memOpsStall = false; // allows for memory translation for new op after this one gets sent through
					} else {
						EX_latch->stage_stall = true;	//if mshr is full
						mshr_full = true;

						memOpsStall = true; // stop access to memory translation
					}
				} // end stuff for MEM_LD
			} else if ((EX_latch->op)->mem_type == MEM_ST) {
				if (mshr_full == false) {
					if (latency_count == 0) {
						latency_count = KNOB(KNOB_DCACHE_LATENCY)->getValue();
						EX_latch->stage_stall = true;

						memOpsStall = true; // stop access to memory translation
					}
					latency_count--;
					if (latency_count == 0) {
						if (dcache_access((EX_latch->op)->st_vaddr)) {
							dcache_hit_count++;
							EX_latch->op_valid = false;
							EX_latch->stage_stall = false;

							fill_retire_queue(EX_latch->op); //not really a broadcast, just using available function

							memOpsStall = false; // allows for memory translation for new op after this one gets sent through
						} else {
							dcache_miss_count++;
							if (main_memory->store_store_forwarding(
									EX_latch->op)) //check if store load fwding is possible
									{
								store_store_forwarding_count++;
								EX_latch->op_valid = false;
								EX_latch->stage_stall = false;

								fill_retire_queue(EX_latch->op); //not really a broadcast, just using available function

								memOpsStall = false; // allows for memory translation for new op after this one gets sent through
							} else if (main_memory->check_piggyback(
									EX_latch->op) == false) {
								if (main_memory->insert_mshr(EX_latch->op)
										== true) //if cannot be piggybacked, try inserting a new entry
										{
									EX_latch->op_valid = false;
									EX_latch->stage_stall = false;

									mshr_full = false;

									memOpsStall = false; // allows for memory translation for new op after this one gets sent through
								} else {
									EX_latch->stage_stall = true; //if mshr is full
									mshr_full = true;

									memOpsStall = true; // stop access to memory translation
								}
							} else {
								EX_latch->op_valid = false;
								EX_latch->stage_stall = false;

								memOpsStall = false; // allows for memory translation for new op after this one gets sent through
							}
						}
					}
				} else {
					if (main_memory->insert_mshr(EX_latch->op) == true)	//if cannot be piggybacked, try inserting a new entry
							{
						EX_latch->op_valid = false;
						EX_latch->stage_stall = false;
						mshr_full = false;

						memOpsStall = false; // allows for memory translation for new op after this one gets sent through
					} else {
						EX_latch->stage_stall = true;	//if mshr is full
						mshr_full = true;

						memOpsStall = true; // stop access to memory translation
					}
				}
			} else {
				EX_latch->op_valid = false;
				EX_latch->stage_stall = false;

				fill_retire_queue(EX_latch->op); //not really a broadcast, just using available function

				//memOpsStall = true; // allows for memory translation for new op after this one gets sent through
			} // if memory type is ld, st, or other

		} // end if ((KNOB(KNOB_ENABLE_VMEM)->getValue() == FALSE) || (KNOB(KNOB_ENABLE_VMEM)->getValue() == FALSE && TLBhit == TRUE))
	} // end if (EX_latch->op_valid == true)
}

void EX_stage() {
	/* you must complete the function */
	static int ex_latency = 0;
	if (ID_latch->op_valid == true) {

		if (EX_latch->stage_stall == false) {

			if (ex_latency == 0)
				ex_latency = get_op_latency(ID_latch->op); //check execution latency of op
			ex_latency--;
			if (ex_latency == 0) {
				ID_latch->op_valid = false;	//set op as invalid for previous stage since op is already transferred
				ID_latch->stage_stall = false;	//if stage stalled remove stall
				EX_latch->op = ID_latch->op;	//transfer pointer to next stage
				EX_latch->op_valid = true;	// set op as valid for the stage

			} else
				ID_latch->stage_stall = true;
		} else
			ID_latch->stage_stall = true;
	}
}

void ID_stage() {
	/* you must complete the function */
#define NUM_SRC_1			((FE_latch->op)->num_src==1)
#define NUM_SRC_2			((FE_latch->op)->num_src==2)
#define SRC(x)				(FE_latch->op)->src[x]
#define SRC_INVALID(x)		!(register_file[SRC(x)].valid)

	if (FE_latch->op_valid == true)	//check if inst lies in the FE latch
			{
		if ((NUM_SRC_1 && (SRC_INVALID(0)))
				|| (NUM_SRC_2 && (SRC_INVALID(0) || SRC_INVALID(1))))//check for source-dest clash
				{
			if (ID_latch->stage_stall == false) {
				if (KNOB(KNOB_USE_BPRED)->getValue() == FALSE) {
					if ((FE_latch->op)->cf_type >= CF_BR)
						control_hazard_count++;
				} else {
					if ((FE_latch->op)->cf_type
							== CF_CBR&& (FE_latch->op)->mispredictedBranch == TRUE)
						control_hazard_count++;
				}

				data_hazard_count++;
			}
			FE_latch->stage_stall = true;
		} else {
			FE_latch->stage_stall = false;

			if (ID_latch->stage_stall == false)	//transfer op only if ID stage is not stalled
					{
				FE_latch->op_valid = false;
				ID_latch->op = FE_latch->op;
				ID_latch->op_valid = true;
				if ((FE_latch->op)->dst != -1)//if dest is valid then mark the specific register as busy/invalid and indicate which op writes to the reg
						{
					register_file[(FE_latch->op)->dst].valid = false;
					reg_writing_ops[(FE_latch->op)->dst] =
							FE_latch->op->inst_id; // in WB stage, clear valid flag of register only if this is the op
				}
				// Check if using branch predictor
				if (KNOB(KNOB_USE_BPRED)->getValue() == TRUE) {
					if ((FE_latch->op)->cf_type
							== CF_CBR&& (FE_latch->op)->mispredictedBranch == TRUE) {
						control_hazard_count++;
						FE_latch->stage_stall = true;
					}
				} else {

					if ((FE_latch->op)->cf_type >= CF_BR) {
						control_hazard_count++;
						FE_latch->stage_stall = true;
					}
				}
			} else {
				FE_latch->stage_stall = true;
			}
		}
	}

}

void FE_stage() {
	/* only part of FE_stage function is implemented */
	/* please complete the rest of FE_stage function */

	if (FE_latch->stage_stall == false)	//check if no pipeline stalled
			{
		Op* new_op = get_free_op();	//get a placeholder op out of the pool
		if (get_op(new_op))	//copy op from trace into the placeholder op
				{
			// check if using branch predictor
			if (KNOB(KNOB_USE_BPRED)->getValue() == TRUE) {
				if (new_op->cf_type == CF_CBR && new_op->opcode == OP_CF) {
					//access branch predictor
					int branPredDir = bpred_access(branchpred,
							new_op->instruction_addr);

					// stall Fetch if there is a misprediction
					if (branPredDir != new_op->actually_taken) {
						new_op->mispredictedBranch = TRUE;
						//stall pipeline
						FE_latch->stage_stall = true;
						bpred_mispred_count++;
						//std::cout << "Misprediction" << endl;
					} else {
						bpred_okpred_count++;
						//std::cout << "OK prediction" << endl;
					}

					//update branch predictor
					bpred_update(branchpred, new_op->instruction_addr,
							branPredDir, new_op->actually_taken);

				}
			}
			FE_latch->op = new_op;
			FE_latch->op_valid = true;
		} else
			free_op(new_op);
	}

	//DO NOT modify following line
	//   next_pc = pc + op->inst_size;  // you need this code for building a branch predictor

}

void init_latches() {
	MEM_latch = new pipeline_latch();
	EX_latch = new pipeline_latch();
	ID_latch = new pipeline_latch();
	FE_latch = new pipeline_latch();

	MEM_latch->op = NULL;
	EX_latch->op = NULL;
	ID_latch->op = NULL;
	FE_latch->op = NULL;

	/* you must set valid value correctly  */
	MEM_latch->op_valid = false;
	EX_latch->op_valid = false;
	ID_latch->op_valid = false;
	FE_latch->op_valid = false;

	MEM_latch->stage_stall = false;
	EX_latch->stage_stall = false;
	ID_latch->stage_stall = false;
	FE_latch->stage_stall = false;
}

bool icache_access(ADDRINT addr) { /** please change uint32_t to ADDRINT NEW-LAB2 */

	/* For Lab #1, you assume that all I-cache hit */
	bool hit = FALSE;
	if (KNOB(KNOB_PERFECT_ICACHE)->getValue())
		hit = TRUE;
	return hit;
}

bool dcache_access(ADDRINT addr) { /** please change uint32_t to ADDRINT NEW-LAB2 */
	/* For Lab #1, you assume that all D-cache hit */
	/* For Lab #2, you need to connect cache here */   // NEW-LAB2
	bool hit = FALSE;
	if (KNOB(KNOB_PERFECT_DCACHE)->getValue())
		hit = TRUE;
	else {
		hit = cache_access(data_cache, addr);
	}
	return hit;
}

//fn added by apk
void init_register_file() {
	for (int i = 0; i < NUM_REG; i++)
		register_file[i].valid = true;	//all registers are initially available
}

void fill_retire_queue(Op* op)             // NEW-LAB2 
		{                                          // NEW-LAB2
	/* you must complete the function */                             // NEW-LAB2
	// mem ops are done.  move the op into WB stage   // NEW-LAB2
	//MEM_latch->op_queue.push_back(op);
	//MEM_latch->op_valid = true;
	// if conditional branch, install into the TLB
	if (KNOB(KNOB_ENABLE_VMEM)->getValue() == TRUE) {
		if (op->opcode == OP_DUMMY) {
			// insert into install TLB queue
			TLBinstallQueue.push_back(op);

			//std::cout << "Returned dummy op from MSHR" << endl;
		} else {
			MEM_latch->op_queue.push_back(op);
			MEM_latch->op_valid = true;
		}
	} else {
		MEM_latch->op_queue.push_back(op);
		MEM_latch->op_valid = true;
	}
}

void installInTLB() {
	while (!TLBinstallQueue.empty()) {
		Op* op = TLBinstallQueue.front();
		uint64_t vpn;
		if ((EX_latch->op)->mem_type == MEM_LD)
			vpn = getVPN((EX_latch->op)->ld_vaddr);	//add for stores also
		if ((EX_latch->op)->mem_type == MEM_ST)
			vpn = getVPN((EX_latch->op)->st_vaddr);	//add for stores also

		uint64_t pfn = vmem_vpn_to_pfn(vpn, 0);
		tlb_install(dtlb, vpn, 0, pfn);
		TLBinstallQueue.pop_front();
		// remove stall
		EX_latch->stage_stall = false;
		EX_latch->op_valid = true;
		TLBMSHRstall = false;
		//std::cout << "installed " << op->inst_id << " into TLB" << endl;
	}

}

bool pipeline_latches_empty() {
	if (FE_latch->op_valid == false && ID_latch->op_valid == false
			&& EX_latch->op_valid == false && MEM_latch->op_valid == false)
		return true;
	else
		return false;
}

uint64_t virtualToPhysical(ADDRINT vaddr, uint64_t pfn) {
	uint64_t paddr = pfn * (KNOB(KNOB_VMEM_PAGE_SIZE)->getValue())
			+ vaddr % (KNOB(KNOB_VMEM_PAGE_SIZE)->getValue());
	return paddr;
}

uint64_t getPTE(ADDRINT vaddr) {
	uint64_t vpn = getVPN(vaddr);
	uint64_t pte = vmem_get_pteaddr(vpn, 0); // pteaddr
	return pte;
}

uint64_t getVPN(ADDRINT vaddr) {
	uint64_t pageOffsetsize = LOG2(KNOB(KNOB_VMEM_PAGE_SIZE)->getValue());
	uint64_t vpn = vaddr >> pageOffsetsize;
	return vpn;
}
