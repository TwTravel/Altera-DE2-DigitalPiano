module mux_16to1_1bit(i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15, s0, s1, s2, s3, out);

	input i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11, i12, i13, i14, i15, s0, s1, s2, s3;
	output out;

	wire ns0, ns1, ns2, ns3;
	not not0(ns0, s0);
	not not1(ns1, s1);
	not not2(ns2, s2);
	not not3(ns3, s3);

	wire t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;

	and and0(t0, i0, ns0, ns1, ns2, ns3);
	and and1(t1, i1, s0, ns1, ns2, ns3);
	and and2(t2, i2, ns0, s1, ns2, ns3);
	and and3(t3, i3, s0, s1, ns2, ns3);
	and and4(t4, i4, ns0, ns1, s2, ns3);
	and and5(t5, i5, s0, ns1, s2, ns3);
	and and6(t6, i6, ns0, s1, s2, ns3);
	and and7(t7, i7, s0, s1, s2, ns3);
	and and8(t8, i8, ns0, ns1, ns2, s3);
	and and9(t9, i9, s0, ns1, ns2, s3);
	and and10(t10, i10, ns0, s1, ns2, s3);
	and and11(t11, i11, s0, s1, ns2, s3);
	and and12(t12, i12, ns0, ns1, s2, s3);
	and and13(t13, i13, s0, ns1, s2, s3);
	and and14(t14, i14, ns0, s1, s2, s3);
	and and15(t15, i15, s0, s1, s2, s3);

	or or0(out, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15);

endmodule