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
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GProp_GProps.hxx>
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
#include <ShapeAnalysis_Surface.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt2d.hxx>

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

        gp_Pnt shapeCenter(0.0, 0.0, 0.0);
        bool hasShapeCenter = false;
        {
            Bnd_Box shapeBounds;
            try { BRepBndLib::Add(shape, shapeBounds, false); }
            catch (...) {
                try { BRepBndLib::Add(shape, shapeBounds, true); }
                catch (...) {}
            }
            if (!shapeBounds.IsVoid()) {
                double xmin, ymin, zmin, xmax, ymax, zmax;
                shapeBounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                shapeCenter.SetCoord((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
                hasShapeCenter = true;
            }
        }

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

            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            Handle(ShapeAnalysis_Surface) sas;
            if (!surface.IsNull()) {
                sas = new ShapeAnalysis_Surface(surface);
            }

            gp_Vec planarRenderNormal;
            bool hasPlanarRenderNormal = false;
            try {
                BRepAdaptor_Surface adaptor(face, Standard_True);
                if (adaptor.GetType() == GeomAbs_Plane) {
                    gp_Dir normalDir = adaptor.Plane().Axis().Direction();
                    if (face.Orientation() == TopAbs_REVERSED) {
                        normalDir.Reverse();
                    }
                    planarRenderNormal = gp_Vec(normalDir);

                    if (hasShapeCenter) {
                        GProp_GProps props;
                        BRepGProp::SurfaceProperties(face, props);
                        const gp_Pnt faceCenter = props.CentreOfMass();
                        gp_Vec outwardHint(shapeCenter, faceCenter);
                        if (outwardHint.SquareMagnitude() > 1.0e-14 &&
                            planarRenderNormal.Dot(outwardHint) < 0.0) {
                            planarRenderNormal.Reverse();
                        }
                    }
                    hasPlanarRenderNormal = planarRenderNormal.SquareMagnitude() > 1.0e-18;
                }
            }
            catch (...) {
                hasPlanarRenderNormal = false;
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

                if (!sas.IsNull()) {
                    const double UV_SCALE = 1.0;
                    gp_Pnt2d uv_params = sas->ValueOfUV(vertex, 1e-6);
                    out.uvs.push_back(static_cast<float>(uv_params.X() * UV_SCALE));
                    out.uvs.push_back(static_cast<float>(uv_params.Y() * UV_SCALE));
                }
                else {
                    out.uvs.push_back(0.0f);
                    out.uvs.push_back(0.0f);
                }

                vertex.Transform(transformation);

                //                       (nx, ny, nz)    TShort_Array1OfShortReal
                Standard_Integer normIndex = (i - nodes.Lower()) * 3 + normals.Lower(); //                  normals
                if (normIndex + 2 > normals.Upper()) { //                
                    Message::SendFail() << "TriangulateShape: Normal index out of bounds.";
                    continue; //                                      
                }
                gp_Dir normal(normals(normIndex), normals(normIndex + 1), normals(normIndex + 2));

                // -- START OF MODIFICATION --
                // --- FIX: Correctly transform the normal direction ---
                // The previous method `normal.Transform(transformation)` was incorrect
                // because it applied translation to a direction vector, corrupting it.
                // The correct way is to apply only the rotational part of the transform.
                // We do this by converting the direction to a vector, transforming it
                // (which ignores translation for gp_Vec), and converting back to a direction.
                if (!location.IsIdentity()) {
                    gp_Vec normalVec(normal.XYZ());
                    normalVec.Transform(transformation);
                    normal = gp_Dir(normalVec);
                }
                // Invert normal for reversed faces for correct lighting
                if (face.Orientation() == TopAbs_REVERSED) {
                    normal.Reverse();
                }
                if (hasPlanarRenderNormal) {
                    normal = gp_Dir(planarRenderNormal);
                }
                // --- END FIX ---
                // -- END OF MODIFICATION --

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

                auto readVertex = [&](unsigned int idx) {
                    const size_t base = static_cast<size_t>(idx) * 3;
                    return gp_Pnt(out.vertices[base + 0], out.vertices[base + 1], out.vertices[base + 2]);
                };
                auto readNormal = [&](unsigned int idx) {
                    const size_t base = static_cast<size_t>(idx) * 3;
                    return gp_Vec(out.normals[base + 0], out.normals[base + 1], out.normals[base + 2]);
                };

                const gp_Pnt p1 = readVertex(idx1);
                const gp_Pnt p2 = readVertex(idx2);
                const gp_Pnt p3 = readVertex(idx3);
                gp_Vec geometricNormal = gp_Vec(p1, p2).Crossed(gp_Vec(p1, p3));
                gp_Vec shadingNormal = readNormal(idx1) + readNormal(idx2) + readNormal(idx3);
                if (hasPlanarRenderNormal) {
                    shadingNormal = planarRenderNormal;
                }
                if (geometricNormal.SquareMagnitude() > 1.0e-18 &&
                    shadingNormal.SquareMagnitude() > 1.0e-18 &&
                    geometricNormal.Dot(shadingNormal) < 0.0) {
                    std::swap(idx2, idx3);
                }

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
