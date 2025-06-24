//===- SVFGBuilder.cpp -- SVFG builder----------------------------------------//
//
//                     SVF: Static Value-Flow Analysis
//
// Copyright (C) <2013-2017>  <Yulei Sui>
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
 * SVFGBuilder.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: Yulei Sui
 */
#include "MSSA/SVFGBuilder.h"
#include "Graphs/CallGraph.h"
#include "Graphs/SVFG.h"
#include "MSSA/MemSSA.h"
#include "Util/Options.h"
#include "Util/SVFUtil.h"
#include "WPA/Andersen.h"
#include "Graphs/GraphWriter.h"

using namespace SVF;
using namespace SVFUtil;

SVFG* SVFGBuilder::buildPTROnlySVFG(BVDataPTAImpl* pta)
{
    return this->SVFGWithPostOpts ? build(pta, VFG::PTRONLYSVFG_OPT)
           : build(pta, VFG::PTRONLYSVFG);
}

SVFG* SVFGBuilder::buildFullSVFG(BVDataPTAImpl* pta)
{
    return this->SVFGWithPostOpts ? build(pta, VFG::FULLSVFG_OPT)
           : build(pta, VFG::FULLSVFG);
}
//kbkang-MODIFIED
std::string escapeDotString(const std::string& raw) {
    std::string escaped;
    for (char c : raw) {
        switch (c) {
            case '\"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\t': escaped += "    "; break; //
            case '\r': break; 
            default: escaped += c;
        }
    }
    return escaped;
}

//yshwang
std::string trim_debug(const std::string& raw){
    auto pos = raw.find('{');
    std::string trimmed;
    if (pos == std::string::npos) {
        trimmed = raw; // No '{' found, use the whole string
    } else {
        trimmed = raw.substr(0, pos);
    }
    return trimmed;

}

void SVFGBuilder::dumpSubgraphFromNode(SVFGNode* startNode, const std::string& outFile) {
    std::set<const SVFGNode*> visited;
    std::queue<const SVFGNode*> worklist;
    worklist.push(startNode);
    visited.insert(startNode);
    SVFUtil::outs() << "Writing Filtered SVFG graph to .dot ...\n";
    std::string outFile2 = "value_integrated_file.txt";
    std::ofstream out(outFile);
    std::ofstream out2(outFile2);
    out << "digraph SVFG_Subgraph {\n";

    // int kind;

    while (!worklist.empty()) {
        const SVFGNode* node = worklist.front();
        worklist.pop();

        // if(PHIVFGNode::classof(node) || MSSAPHISVFGNode::classof(node)){ 
        //     kind = node->getNodeKind();
        //     switch(kind) 
        //     {
        //     // NodeKind 48,49,50 are for PHIVFGNode(specifically, TPhi, TIntraPhi, TInterPhi)
        //     case 48: // TPhi
        //         out << "  Node" << node->getId() << " [label=PHINode];\n";
        //         break;
        //     case 49: // TIntraPhi
        //         out << "  Node" << node->getId() << " [label=IntraPHINode];\n";
        //         break;
        //     case 50: // TInterPhi
        //         out << "  Node" << node->getId() << " [label=InterPHINode];\n";
        //         break;

        //     // NodeKind 55,56,57 are for MSSAPHISVFGNode(specifically, MPhi, MIntraPhi, MInterPhi)
        //     case 55: // Phi
        //         out << "  Node" << node->getId() << " [label=MPHINode];\n";
        //         break;
        //     case 56: // MIntraPhi
        //         out << "  Node" << node->getId() << " [label=MIntraPHINode];\n";
        //         break;
        //     case 57: // MInterPhi
        //         out << "  Node" << node->getId() << " [label=MInterPHINode];\n";
        //         break;
        //     default:
        //         out << "  Node" << node->getId() << " isPHIVFGNodeKinds or is MSSAPHISVFGNodebut, not handled: " << kind << "\n";
        //         break;
        //     }
        // }
        // else{
        //     out << "  Node" << node->getId() << " [label=\"" << escapeDotString(node->toString()) << "\"];\n";
        // }

        out << "  Node" << node->getId() << " [label=\"" << escapeDotString(node->toString()) << "\"];\n";
        out2 << "  Node" << node->getId() << "\t" << escapeDotString(node->toString()) << "\n";

        for (const SVFGEdge* edge : node->getOutEdges()) {
            const SVFGNode* succ = edge->getDstNode();
	        out << "  Node" << node->getId() << " -> Node" << succ->getId() << ";\n";
            if (visited.insert(succ).second) {
                worklist.push(succ);
            }
        }
    }

    out << "}\n";
    out2 << "\n";
    out.close();
    out2.close();
}

/*!
 * Create SVFG
 */
void SVFGBuilder::buildSVFG()
{
    svfg->buildSVFG();
    
    for (auto it = svfg->begin(); it != svfg->end(); ++it) {
        SVFGNode* node = it->second;
        const SVFVar* svfVar = node->getValue();
        if (!svfVar) continue;
        
        if(!SVFUtil::isa<GepObjVar>(svfVar) && !SVFUtil::isa<GepValVar>(svfVar) && !SVFUtil::isa<DummyObjVar>(svfVar) 
        && !SVFUtil::isa<DummyValVar>(svfVar) && !SVFUtil::isa<BlackHoleValVar>(svfVar)){
            std::string varStr = svfVar->valueOnlyToString();
            std::string trim_varStr = trim(trim_debug(varStr));
            // SVFUtil::outs() << "[DEBUG] Value extracted from SVF value : " << trim_varStr << "\n";
            if (secretAnnotatedVals.count(trim_varStr) && node->getNodeKind() == 43) { // Node number 43 is for AddrVFGNode
                SVFUtil::outs() << "[SecretNode] Match found: Node Kind=" << node->getNodeKind() << "\nNode Id=" <<node->getId()
                << "\n  ==> " << trim_varStr << "\n";
                targetNode = node;
            }
            // SVFUtil::outs() << "[Compare] trim_varStr : [" << trim_varStr << "]\n";
            // for(const auto& secret : secretAnnotatedVals){
            //     SVFUtil::outs() << "[Compare] secret : [" << secret << "]\n";
            //     SVFUtil::outs() << (trim_varStr == secret ? "Match" : "Nope") << "\n";
            // }
        }
    }
    
    if (targetNode) {
        std::string outDotFile = "subgraph_from_secret_node.dot";
        dumpSubgraphFromNode(targetNode, outDotFile);
        SVFUtil::outs() << "[Subgraph] Dumped reachable subgraph from Node " << targetNode->getId()
                        << " to " << outDotFile << "\n";
    } else {
        SVFUtil::outs() << "[Subgraph] No secret target node found; nothing dumped.\n";
    }
//kbkang-MODIFIED
}

/// Create DDA SVFG
SVFG* SVFGBuilder::build(BVDataPTAImpl* pta, VFG::VFGK kind)
{

    auto mssa = buildMSSA(
                    pta, (VFG::PTRONLYSVFG == kind || VFG::PTRONLYSVFG_OPT == kind));

    DBOUT(DGENERAL, outs() << pasMsg("Build Sparse Value-Flow Graph \n"));
    if (kind == VFG::FULLSVFG_OPT || kind == VFG::PTRONLYSVFG_OPT)
        svfg = std::make_unique<SVFGOPT>(std::move(mssa), kind);
    else
        svfg = std::unique_ptr<SVFG>(new SVFG(std::move(mssa), kind));
    buildSVFG();

    /// Update call graph using pre-analysis results
    if (SVFGWithIndCall)
        svfg->updateCallGraph(pta);

    if (svfg->getMSSA()->getPTA()->printStat())
        svfg->performStat();

    if (Options::DumpVFG()){
        svfg->dump("svfg_final");
	//svfg->dump_filtered("filtered_svfg",targetNode);
    }

    return svfg.get();
}

/*!
 * Release memory
 */
void SVFGBuilder::releaseMemory()
{
    svfg->clearMSSA();
}

std::unique_ptr<MemSSA> SVFGBuilder::buildMSSA(BVDataPTAImpl* pta,
        bool ptrOnlyMSSA)
{

    DBOUT(DGENERAL, outs() << pasMsg("Build Memory SSA \n"));

    auto mssa = std::make_unique<MemSSA>(pta, ptrOnlyMSSA);

    const CallGraph* svfirCallGraph = PAG::getPAG()->getCallGraph();
    for (const auto& item : *svfirCallGraph)
    {

        const FunObjVar* fun = item.second->getFunction();
        if (isExtCall(fun))
            continue;

        mssa->buildMemSSA(*fun);
    }

    mssa->performStat();
    if (Options::DumpMSSA())
    {
        mssa->dumpMSSA();
    }

    return mssa;
}
