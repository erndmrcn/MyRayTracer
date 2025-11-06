import ParsingKit

public struct MeshInstance: Sendable {
    public var baseIndex: Int
    public var transform: Mat4
    public var invTransform: Mat4
    public var worldBounds: AABB
    public var materialIndex: Int
}
