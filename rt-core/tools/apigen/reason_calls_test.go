package main

import "go/ast"

type reasonValue struct {
	expression ast.Expr
	index      int
}

type reasonCalls struct {
	sources   *reasonSources
	methods   map[string][]*ast.FuncDecl
	bindings  map[*ast.Object][]reasonValue
	fields    map[*ast.Ident][]reasonValue
	closures  map[*ast.FuncLit]*ast.FuncDecl
	functions []*ast.FuncDecl
}

// newReasonCalls joins all writes to each declared function field and propagates
// helper arguments to a fixed point, preventing source order from hiding callbacks.
func newReasonCalls(sources *reasonSources, methods map[string][]*ast.FuncDecl) *reasonCalls {
	sources.resolveLiteralTypes()
	c := &reasonCalls{sources: sources, methods: methods, bindings: map[*ast.Object][]reasonValue{},
		fields: map[*ast.Ident][]reasonValue{}, closures: map[*ast.FuncLit]*ast.FuncDecl{}}
	var calls []*ast.CallExpr
	for _, file := range sources.trees {
		ast.Inspect(file, func(node ast.Node) bool {
			switch node := node.(type) {
			case *ast.FuncDecl:
				c.functions = append(c.functions, node)
			case *ast.FuncLit:
				c.closures[node] = &ast.FuncDecl{Type: node.Type, Body: node.Body}
				c.functions = append(c.functions, c.closures[node])
			case *ast.StructType:
				for _, field := range node.Fields.List {
					if _, ok := sources.underlyingType(field.Type, map[ast.Expr]bool{}).(*ast.FuncType); ok {
						for _, name := range field.Names {
							c.fields[name] = nil
						}
					}
				}
			case *ast.CallExpr:
				calls = append(calls, node)
			}
			return true
		})
	}
	for _, file := range sources.trees {
		ast.Inspect(file, func(node ast.Node) bool {
			switch node := node.(type) {
			case *ast.AssignStmt:
				for i, target := range node.Lhs {
					c.bind(target, node.Rhs, i)
				}
			case *ast.ValueSpec:
				for i, target := range node.Names {
					c.bind(target, node.Values, i)
				}
			case *ast.CompositeLit:
				structure, ok := sources.underlyingType(node.Type, map[ast.Expr]bool{}).(*ast.StructType)
				if !ok {
					break
				}
				var names []*ast.Ident
				for _, field := range structure.Fields.List {
					if len(field.Names) == 0 {
						names = append(names, nil) // Embedded fields still occupy a positional slot.
					}
					names = append(names, field.Names...)
				}
				for i, element := range node.Elts {
					var target *ast.Ident
					if pair, ok := element.(*ast.KeyValueExpr); ok {
						if key, ok := pair.Key.(*ast.Ident); ok {
							for _, name := range names {
								if name != nil && name.Name == key.Name {
									target = name
								}
							}
						}
						element = pair.Value
					} else if i < len(names) {
						target = names[i]
					}
					if _, ok := c.fields[target]; ok {
						c.fields[target], _ = appendReasonBinding(c.fields[target], reasonValue{element, 0})
					}
				}
			}
			return true
		})
	}
	for _, conservative := range []bool{false, true} {
		for changed := true; changed; {
			changed = false
			for _, call := range calls {
				functions, _ := c.resolve(reasonValue{call.Fun, 0}, map[reasonValue]bool{})
				if conservative {
					functions = c.called(call)
				}
				for _, function := range functions {
					index := 0
					for _, field := range function.Type.Params.List {
						for _, name := range field.Names {
							changed = c.bind(name, call.Args, index) || changed
							index++
						}
						if len(field.Names) == 0 {
							index++
						}
					}
				}
			}
		}
	}
	return c
}

func appendReasonBinding(inputs []reasonValue, input reasonValue) ([]reasonValue, bool) {
	for _, existing := range inputs {
		if existing == input {
			return inputs, false
		}
	}
	return append(inputs, input), true
}

func (c *reasonCalls) bind(target ast.Expr, expressions []ast.Expr, index int) bool {
	if len(expressions) == 0 {
		return false
	}
	var input reasonValue
	if index < len(expressions) {
		input = reasonValue{expressions[index], 0}
	} else if len(expressions) == 1 {
		input = reasonValue{expressions[0], index}
	} else {
		return false
	}
	changed := false
	if object := c.sources.object(target); object != nil {
		c.bindings[object], changed = appendReasonBinding(c.bindings[object], input)
	} else if selector, ok := target.(*ast.SelectorExpr); ok {
		for _, field := range c.selectedFields(selector) {
			var added bool
			c.fields[field], added = appendReasonBinding(c.fields[field], input)
			changed = added || changed
		}
	}
	return changed
}

// selectedFields uses declaration identity for concrete receivers; unresolved
// receivers retain every matching field so uncertain dispatch cannot erase a label.
func (c *reasonCalls) selectedFields(selector *ast.SelectorExpr) []*ast.Ident {
	var found []*ast.Ident
	if structure, ok := c.sources.underlyingType(selector.X, map[ast.Expr]bool{}).(*ast.StructType); ok {
		for _, field := range c.sources.selectedStructFields(structure, selector.Sel.Name, map[*ast.StructType]bool{}) {
			for _, name := range field.Names {
				if name.Name == selector.Sel.Name {
					if _, ok := c.fields[name]; ok {
						found = append(found, name)
					}
				}
			}
		}
		return found
	}
	for field := range c.fields {
		if field.Name == selector.Sel.Name {
			found = append(found, field)
		}
	}
	return found
}

func (c *reasonCalls) called(call *ast.CallExpr) []*ast.FuncDecl {
	found, complete := c.resolve(reasonValue{call.Fun, 0}, map[reasonValue]bool{})
	if !complete || len(found) == 0 {
		signature, _ := c.sources.underlyingType(call.Fun, map[ast.Expr]bool{}).(*ast.FuncType)
		if signature != nil {
			for _, function := range c.functions {
				if reasonFieldCount(signature.Params) == reasonFieldCount(function.Type.Params) &&
					reasonFieldCount(signature.Results) == reasonFieldCount(function.Type.Results) {
					found = append(found, function)
				}
			}
		}
	}
	return found
}

// resolve follows aliases, field writes and function-returning helpers. Unknown
// field inputs retain arity-compatible bodies instead of proving them private.
func (c *reasonCalls) resolve(input reasonValue, visiting map[reasonValue]bool) ([]*ast.FuncDecl, bool) {
	if visiting[input] {
		return nil, true // A cycle adds no target; other writes can still resolve it.
	}
	visiting[input] = true
	defer delete(visiting, input)
	var found []*ast.FuncDecl
	complete := true
	follow := func(inputs []reasonValue) {
		if len(inputs) == 0 {
			complete = false
		}
		for _, next := range inputs {
			functions, known := c.resolve(next, visiting)
			found = append(found, functions...)
			complete = complete && known
		}
	}
	if object := c.sources.object(input.expression); object != nil {
		if function, ok := object.Decl.(*ast.FuncDecl); ok {
			return []*ast.FuncDecl{function}, true
		}
		follow(c.bindings[object])
		return found, complete
	}
	switch expression := input.expression.(type) {
	case *ast.FuncLit:
		return []*ast.FuncDecl{c.closures[expression]}, true
	case *ast.ParenExpr:
		return c.resolve(reasonValue{expression.X, input.index}, visiting)
	case *ast.SelectorExpr:
		fields := c.selectedFields(expression)
		for _, field := range fields {
			follow(c.fields[field])
		}
		if len(fields) > 0 {
			if _, concrete := c.sources.underlyingType(expression.X, map[ast.Expr]bool{}).(*ast.StructType); concrete {
				return found, complete
			}
		}
		receiver := c.sources.declaredType(expression.X, map[ast.Expr]bool{})
		for _, method := range c.methods[expression.Sel.Name] {
			if receiver == nil || receiver == c.sources.declaredType(method.Recv.List[0].Type, map[ast.Expr]bool{}) {
				found = append(found, method)
			}
		}
		return found, complete && len(found) > 0
	case *ast.CallExpr:
		functions, known := c.resolve(reasonValue{expression.Fun, 0}, visiting)
		complete = known
		for _, function := range functions {
			var results []reasonValue
			if function.Body != nil {
				ast.Inspect(function.Body, func(node ast.Node) bool {
					if _, nested := node.(*ast.FuncLit); nested {
						return false
					}
					if statement, ok := node.(*ast.ReturnStmt); ok {
						if input.index < len(statement.Results) {
							results = append(results, reasonValue{statement.Results[input.index], 0})
						} else if len(statement.Results) == 1 {
							results = append(results, reasonValue{statement.Results[0], input.index})
						} else if len(statement.Results) == 0 && function.Type.Results != nil {
							index := 0
							for _, field := range function.Type.Results.List {
								for _, name := range field.Names {
									if index == input.index {
										results = append(results, reasonValue{name, 0})
									}
									index++
								}
							}
						}
					}
					return true
				})
			}
			follow(results)
		}
		return found, complete
	}
	return nil, false
}

func reasonFieldCount(fields *ast.FieldList) int {
	count := 0
	if fields != nil {
		for _, field := range fields.List {
			if len(field.Names) == 0 {
				count++
			} else {
				count += len(field.Names)
			}
		}
	}
	return count
}

// underlyingType resolves declared struct and function types across adapter
// imports and constructors, preventing field selectors from becoming method calls.
func (s *reasonSources) underlyingType(expression ast.Expr, visiting map[ast.Expr]bool) ast.Expr {
	if expression == nil || visiting[expression] {
		return nil
	}
	visiting[expression] = true
	defer delete(visiting, expression)
	if object := s.object(expression); object != nil {
		switch declaration := object.Decl.(type) {
		case *ast.TypeSpec:
			return s.underlyingType(declaration.Type, visiting)
		case *ast.Field:
			return s.underlyingType(declaration.Type, visiting)
		case *ast.ValueSpec:
			if declaration.Type != nil {
				return s.underlyingType(declaration.Type, visiting)
			}
			for i, name := range declaration.Names {
				if name.Obj == object && i < len(declaration.Values) {
					return s.underlyingType(declaration.Values[i], visiting)
				}
			}
		case *ast.AssignStmt:
			for i, target := range declaration.Lhs {
				if s.object(target) == object && i < len(declaration.Rhs) {
					return s.underlyingType(declaration.Rhs[i], visiting)
				}
			}
		}
	}
	switch expression := expression.(type) {
	case *ast.StructType, *ast.FuncType, *ast.ArrayType, *ast.MapType:
		return expression
	case *ast.StarExpr:
		return s.underlyingType(expression.X, visiting)
	case *ast.UnaryExpr:
		return s.underlyingType(expression.X, visiting)
	case *ast.ParenExpr:
		return s.underlyingType(expression.X, visiting)
	case *ast.CompositeLit:
		return s.underlyingType(expression.Type, visiting)
	case *ast.IndexExpr:
		switch container := s.underlyingType(expression.X, visiting).(type) {
		case *ast.ArrayType:
			return s.underlyingType(container.Elt, visiting)
		case *ast.MapType:
			return s.underlyingType(container.Value, visiting)
		}
	case *ast.CallExpr:
		if object := s.object(expression.Fun); object != nil {
			if function, ok := object.Decl.(*ast.FuncDecl); ok && function.Type.Results != nil {
				return s.underlyingType(function.Type.Results.List[0].Type, visiting)
			}
		}
		return s.underlyingType(expression.Fun, visiting)
	case *ast.SelectorExpr:
		if structure, ok := s.underlyingType(expression.X, visiting).(*ast.StructType); ok {
			for _, field := range s.selectedStructFields(structure, expression.Sel.Name, map[*ast.StructType]bool{}) {
				for _, name := range field.Names {
					if name.Name == expression.Sel.Name {
						return s.underlyingType(field.Type, visiting)
					}
				}
			}
		}
	}
	return nil
}

// selectedStructFields includes promoted fields without losing declaration
// identity; recursive embedding must not make inventory traversal loop.
func (s *reasonSources) selectedStructFields(structure *ast.StructType, name string, visiting map[*ast.StructType]bool) []*ast.Field {
	if visiting[structure] {
		return nil
	}
	visiting[structure] = true
	defer delete(visiting, structure)
	for _, field := range structure.Fields.List {
		for _, fieldName := range field.Names {
			if fieldName.Name == name {
				return []*ast.Field{field}
			}
		}
	}
	var found []*ast.Field
	for _, field := range structure.Fields.List {
		if len(field.Names) == 0 {
			if embedded, ok := s.underlyingType(field.Type, map[ast.Expr]bool{}).(*ast.StructType); ok {
				found = append(found, s.selectedStructFields(embedded, name, visiting)...)
			}
		}
	}
	return found
}

// resolveLiteralTypes fills elided types in the parsed tree from the enclosing
// container, so a slice or map literal cannot hide an additional field binding.
func (s *reasonSources) resolveLiteralTypes() {
	for _, file := range s.trees {
		var parents []ast.Node
		ast.Inspect(file, func(node ast.Node) bool {
			if node == nil {
				parents = parents[:len(parents)-1]
				return true
			}
			if literal, ok := node.(*ast.CompositeLit); ok && literal.Type == nil && len(parents) > 0 {
				parent := parents[len(parents)-1]
				if _, keyed := parent.(*ast.KeyValueExpr); keyed && len(parents) > 1 {
					parent = parents[len(parents)-2]
				}
				if container, ok := parent.(*ast.CompositeLit); ok {
					switch typ := s.underlyingType(container.Type, map[ast.Expr]bool{}).(type) {
					case *ast.ArrayType:
						literal.Type = typ.Elt
					case *ast.MapType:
						literal.Type = typ.Value
						if pair, ok := parents[len(parents)-1].(*ast.KeyValueExpr); ok && pair.Key == literal {
							literal.Type = typ.Key
						}
					}
				}
			}
			parents = append(parents, node)
			return true
		})
	}
}
