#include "cad_kernel/MeshBuffers.h"
#include "cad_kernel/cad_kernel.h"

// ---                       OpenCascade ---
#include <Standard_Macro.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>
#include <TColgp_Array1OfPnt.hxx>      //                  
#include <TShort_Array1OfShortReal.hxx> //              (           )
#include <TColgp_HArray1OfPnt.hxx>      // Handle                  
#include <TShort_HArray1OfShortReal.hxx>// Handle                     
#include <Poly_Array1OfTriangle.hxx>   //                          
#include <Message.hxx>

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm> // For std::min/max

namespace Urbaxio::CadKernel {

    MeshBuffers TriangulateShape(const TopoDS_Shape& shape,
        double linDefl,
        double angDefl,
        double clampDefl) // Optional clamp for auto-calculated linDefl
    {
        MeshBuffers out;
        if (shape.IsNull()) { /*...*/ return out; }

        // --- 1.                             ---
        if (linDefl <= 0.0) { /* ... (            linDefl        ) ... */ }
        //             linDefl:
        if (linDefl <= 0.0)
        {
            Bnd_Box bb;
            try { BRepBndLib::Add(shape, bb, false); }
            catch (...) {
                try { BRepBndLib::Add(shape, bb, true); }
                catch (...) {}
            }
            if (bb.IsVoid()) { linDefl = 0.1; }
            else {
                double diag = bb.SquareExtent() > 1.e-16 ? std::sqrt(bb.SquareExtent()) : 1.0;
                linDefl = diag * 0.002;
                if (clampDefl > 0) {
                    linDefl = std::min(linDefl, clampDefl);
                }
                linDefl = std::max(linDefl, 1e-6); // Set a minimum threshold to avoid issues with tiny objects
            }
            Message::SendInfo() << "TriangulateShape: Using auto-calculated linear deflection: " << linDefl << ", angular deflection: " << angDefl;
        }

        // --- 2.                     ---
        Standard_ErrorHandler aErrHandler;
        try {
            OCC_CATCH_SIGNALS
                BRepTools::Clean(shape);
            BRepMesh_IncrementalMesh mesher(shape, linDefl, Standard_False, angDefl, Standard_True);
            if (!mesher.IsDone()) { /* ... */ }
            else { /* ... */ }
        }
        catch (Standard_Failure& e) { /* ... */ return out; }
        catch (...) { /* ... */ return out; }

        // --- 3.                                       ---
        TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
        for (; faceExplorer.More(); faceExplorer.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(faceExplorer.Current());
            TopLoc_Location location;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

            if (triangulation.IsNull() || triangulation->NbNodes() == 0 || triangulation->NbTriangles() == 0) {
                continue; //                               
            }

            // --- 4.          /                 ---
            if (!triangulation->HasNormals()) {
                try {
                    triangulation->ComputeNormals();
                }
                catch (...) { continue; }
            }
            if (!triangulation->HasNormals()) { continue; }

            const gp_Trsf& transformation = location.Transformation();

            // ---                  API                        ---
            const Handle(TColgp_HArray1OfPnt) hNodes = triangulation->MapNodeArray();
            const Handle(TShort_HArray1OfShortReal) hNormals = triangulation->MapNormalArray();
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles(); //                                       

            //          ,                     
            if (hNodes.IsNull() || hNormals.IsNull() || hNormals->Length() != hNodes->Length() * 3) {
                Message::SendFail() << "TriangulateShape: Error getting nodes or normals arrays or size mismatch.";
                continue;
            }
            const TColgp_Array1OfPnt& nodes = hNodes->Array1();
            const TShort_Array1OfShortReal& normals = hNormals->Array1(); //                       ShortReal!

            const unsigned int baseVertexIndex = static_cast<unsigned int>(out.vertices.size() / 3);

            // --- 5.                            ---
            for (Standard_Integer i = nodes.Lower(); i <= nodes.Upper(); ++i) {
                gp_Pnt vertex = nodes(i);
                vertex.Transform(transformation);

                //                       (nx, ny, nz)    TShort_Array1OfShortReal
                Standard_Integer normIndex = (i - nodes.Lower()) * 3 + normals.Lower(); //                  normals
                if (normIndex + 2 > normals.Upper()) { //                
                    Message::SendFail() << "TriangulateShape: Normal index out of bounds.";
                    continue; //                                      
                }
                gp_Dir normal(normals(normIndex), normals(normIndex + 1), normals(normIndex + 2));

                // --- FIX: Correctly transform the normal direction ---
                // This applies only the rotational part of the transformation, ignoring any translation.
                // The old method incorrectly treated the normal as a point, corrupting its direction.
                normal.Transform(transformation);

                // Invert normal for reversed faces for correct lighting
                if (face.Orientation() == TopAbs_REVERSED) {
                    normal.Reverse();
                }
                // --- END FIX ---

                out.vertices.push_back(static_cast<float>(vertex.X()));
                out.vertices.push_back(static_cast<float>(vertex.Y()));
                out.vertices.push_back(static_cast<float>(vertex.Z()));
                out.normals.push_back(static_cast<float>(normal.X()));
                out.normals.push_back(static_cast<float>(normal.Y()));
                out.normals.push_back(static_cast<float>(normal.Z()));
            }

            // --- 6.                  ---
            const bool reversed = (face.Orientation() == TopAbs_REVERSED);
            for (Standard_Integer i = triangles.Lower(); i <= triangles.Upper(); ++i) {
                Standard_Integer n1, n2, n3;
                triangles(i).Get(n1, n2, n3);
                unsigned int idx1 = baseVertexIndex + n1 - 1;
                unsigned int idx2 = baseVertexIndex + n2 - 1;
                unsigned int idx3 = baseVertexIndex + n3 - 1;
                unsigned int max_index = baseVertexIndex + nodes.Length() - 1;
                if (idx1 > max_index || idx2 > max_index || idx3 > max_index) { continue; }
                if (reversed) { std::swap(idx2, idx3); }
                out.indices.push_back(idx1);
                out.indices.push_back(idx2);
                out.indices.push_back(idx3);
            }
        }

        Message::SendInfo() << "TriangulateShape: Finished. Vertices: " << out.vertices.size() / 3
            << ", Indices: " << out.indices.size();

        return out;
    }

} // namespace Urbaxio::CadKernel 