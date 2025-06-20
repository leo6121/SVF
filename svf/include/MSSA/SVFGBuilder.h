//===- SVFGBuilder.h -- Building SVFG-----------------------------------------//
//
//                     SVF: Static Value-Flow Analysis
//
// Copyright (C) <2013->  <Yulei Sui>
//

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.

// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//===----------------------------------------------------------------------===//

/*
 * AndersenMemSSA.h
 *
 *  Created on: Oct 27, 2013
 *      Author: Yulei Sui
 */

#ifndef ANDERSENMEMSSA_H_
#define ANDERSENMEMSSA_H_

#include "MemoryModel/PointerAnalysis.h"
#include "Graphs/SVFGOPT.h"
#include "Util/Options.h"

namespace SVF
{

/*!
 * SVFG Builder
 */
class SVFGBuilder
{

public:
    typedef PointerAnalysis::CallSiteSet CallSiteSet;
    typedef PointerAnalysis::CallEdgeMap CallEdgeMap;
    typedef PointerAnalysis::FunctionSet FunctionSet;
    typedef SVFG::SVFGEdgeSetTy SVFGEdgeSet;

    /// Constructor
    explicit SVFGBuilder(bool _SVFGWithIndCall = Options::SVFGWithIndirectCall(), bool _SVFGWithPostOpts = Options::OPTSVFG())
        : svfg(nullptr), SVFGWithIndCall(_SVFGWithIndCall), SVFGWithPostOpts(_SVFGWithPostOpts)
    {
    }

    /// Destructor
    virtual ~SVFGBuilder() = default;

    SVFG* buildPTROnlySVFG(BVDataPTAImpl* pta);
    SVFG* buildFullSVFG(BVDataPTAImpl* pta);
//kbkang-MODIFIED
    std::string trim(const std::string& s) {
	size_t start = s.find_first_not_of(" \t\n\r");
   	size_t end = s.find_last_not_of(" \t\n\r");
	return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

    void setSecretAnnotatedValues(const std::unordered_set<std::string>& vals){
	    secretAnnotatedVals = vals;
    }

    const std::unordered_set<std::string>& getSecretAnnotatedValues() const {
	    return secretAnnotatedVals;
    }
//kbkang-MODIFIED
    /// Get SVFG instance
    inline SVFG* getSVFG() const
    {
        return svfg.get();
    }

    /// Mark feasible VF edge by removing it from set vfEdgesAtIndCallSite
    inline void markValidVFEdge(SVFGEdgeSet& edges)
    {
        for(SVFGEdgeSet::iterator it = edges.begin(), eit = edges.end(); it!=eit; ++it)
            vfEdgesAtIndCallSite.erase(*it);
    }
    /// Return true if this is an VF Edge pre-connected by Andersen's analysis
    inline bool isSpuriousVFEdgeAtIndCallSite(const SVFGEdge* edge)
    {
        return vfEdgesAtIndCallSite.find(const_cast<SVFGEdge*>(edge))!=vfEdgesAtIndCallSite.end();
    }

    /// Build Memory SSA
    virtual std::unique_ptr<MemSSA> buildMSSA(BVDataPTAImpl* pta, bool ptrOnlyMSSA);

protected:
    /// Create a DDA SVFG. By default actualOut and FormalIN are removed, unless withAOFI is set true.
    SVFG* build(BVDataPTAImpl* pta, VFG::VFGK kind);
    /// Can be rewritten by subclasses
    virtual void buildSVFG();
    /// Release global SVFG
    virtual void releaseMemory();
//kbkang-MODIFIED
    virtual void dumpSubgraphFromNode(SVFGNode* startNode, const std::string& outFile);

    std::unordered_set<std::string> secretAnnotatedVals;

    SVFGNode* targetNode = nullptr;
//kbkang-MODIFIED
    /// SVFG Edges connected at indirect call/ret sites
    SVFGEdgeSet vfEdgesAtIndCallSite;
    std::unique_ptr<SVFG> svfg;
    /// SVFG with precomputed indirect call edges
    bool SVFGWithIndCall;
    /// Build optimised version of SVFG
    bool SVFGWithPostOpts;
};

} // End namespace SVF

#endif /* ANDERSENMEMSSA_H_ */
