// /// @overload: Option to change wall positions for blocks of chambers set to move
// ///
// /// @param do_cham_blocks: Pointer array of chamber indexes to move. DEFAULT: true.
// /// @return Status/error codes from @ref WallOperation::_moveConductor()
// uint8_t WallOperation::moveWalls(bool do_cham_blocks)
// {
// 	uint8_t run_status = 0;
// 	uint8_t block_cnt = 0; // counter for number of stages

// 	// Create and array of all chambers set to move
// 	uint8_t cham_all_arr[nCham];
// 	uint8_t n_cham_all = 0;

// 	// Find sand store all chambers flagged for movement
// 	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
// 		if (C[cham_i].bitWallRaiseFlag != 0)
// 			cham_all_arr[n_cham_all++] = cham_i;

// 	// Pass all walls to main version of method
// 	if (!do_cham_blocks)
// 		return _moveWalls(cham_all_arr, n_cham_all);

// 	// TEMP
// 	_Dbg.printMsg(_Dbg.MT::DEBUG, "########### START DEBUG ###########");
// 	_Dbg.printMsg(_Dbg.MT::DEBUG, "_____TEST1: nCham[%d] nChamPerBlock[%d] n_cham_all[%d]", nCham, nChamPerBlock, n_cham_all);

// 	// Store chambers to move next
// 	uint8_t cham_queued_arr[nChamPerBlock];
// 	size_t n_cham_queued = 0;

// 	// Move sets of chambers in stages
// 	for (size_t cham_i = 0; cham_i < n_cham_all; cham_i++)
// 	{
// 		// Store chambers to move next
// 		cham_queued_arr[n_cham_queued++] = cham_all_arr[cham_i];

// 		// TEMP
// 		_Dbg.printMsg(_Dbg.MT::DEBUG, "_____TEST1A: cham_i[%d|%d] n_cham_queued[%d] block_cnt[%d]", cham_i, n_cham_all, n_cham_queued, block_cnt);
// 		_Dbg.printMsg(_Dbg.MT::DEBUG, "_____TEST1B: (cham_i + 1)[%d]    n_cham_all[%d]    {(cham_i + 1) == n_cham_all}[%d]",
// 					  (cham_i + 1), n_cham_all, (cham_i + 1) == n_cham_all);
// 		_Dbg.printMsg(_Dbg.MT::DEBUG, "_____TEST1C: (n_cham_queued == nChamPerBlock || (cham_i + 1) == n_cham_all)[%d]",
// 					  n_cham_queued == nChamPerBlock || (cham_i + 1) == n_cham_all);

// 		// Run walls once max reached
// 		if (n_cham_queued == nChamPerBlock || (cham_i + 1) == n_cham_all)
// 		{
// 			_Dbg.printMsg(_Dbg.MT::DEBUG, "_____________TEST2: cham_queued_arr%s", _Dbg.arrayStr(cham_queued_arr, n_cham_queued));

// 			block_cnt++;
// 			run_status = run_status > 1 ? run_status : _moveWalls(cham_queued_arr, n_cham_queued, block_cnt);

// 			// Reset counter
// 			n_cham_queued = 0;
// 		}
// 	}

// 	_Dbg.printMsg(_Dbg.MT::DEBUG, "########### END DEBUG ###########");

// 	return run_status;
// }